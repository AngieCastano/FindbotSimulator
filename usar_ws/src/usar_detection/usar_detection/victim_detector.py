"""
Nodo victim_detector: detecta víctimas con los sensores del robot y las ubica en el mapa.

Entradas (solo sensores y estimación propia del robot):
  /camera/image, /camera/depth_image, /camera/camera_info   (detectores de la cámara RGB-D)
  /thermal/image (32FC1, °C)                                 (detector térmico, fase 2)
  TF map → frames ópticos de las cámaras (SLAM + modelo del robot)
  /map (mapa del propio robot): descarta evidencia en celdas desconocidas y limita los rayos
       de la cámara térmica, que se triangulan (intersección de rayos desde varias direcciones)
Salidas:
  /victims/markers     (visualization_msgs/MarkerArray) víctimas confirmadas y tentativas
  /victims/detections  (geometry_msgs/PoseArray, frame map) víctimas confirmadas (fusionadas)
  /victims/detections/<detector> (PoseArray) confirmadas por cada detector por separado
  /victims/debug_image (sensor_msgs/Image) imagen con la segmentación superpuesta
  /thermal/debug_image (sensor_msgs/Image) imagen térmica en falso color con las manchas
  log CSV en log_dir   (tiempo, víctima, posición estimada, observaciones, confianza)

Los detectores de signos de vida se eligen con el parámetro `detectors`; cada uno lee sus
parámetros con el prefijo de su nombre (p. ej. color_shape.hsv_lower) y acumula evidencia en su
propio mapa (con el filtro de forma que exija el detector). Un registro común une las víctimas
confirmadas por varios detectores en el mismo lugar y les da una numeración única.
Regla de diseño: este nodo NUNCA lee ground_truth.yaml ni poses de Gazebo.
"""

import csv
from datetime import datetime
import os

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import message_filters
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
import tf2_ros
from usar_detection import color_detector  # noqa: F401  (registra ColorShapeDetector)
from usar_detection import human_shape_detector  # noqa: F401  (registra HumanShapeDetector)
from usar_detection import thermal_detector  # noqa: F401  (registra ThermalDetector)
from usar_detection.bearing_evidence import BearingEvidence
from usar_detection.map_filter import known_cells_mask, ray_extents
from usar_detection.sign_of_life import (create_detector, intrinsics_from_fov, Observation,
                                         ThermalObservation)
from usar_detection.tracker import VictimRegistry, VictimTracker
from visualization_msgs.msg import Marker, MarkerArray

PARAMS = {
    'detectors': ['color_shape'],
    'rgb_topic': '/camera/image',
    'depth_topic': '/camera/depth_image',
    'camera_info_topic': '/camera/camera_info',
    'global_frame': 'map',
    'base_frame': 'base_footprint',
    'processing_rate': 5.0,
    'sync_slop': 0.05,
    'tf_tolerance': 0.1,
    'evidence_cell': 0.1,
    'join_distance': 0.2,
    'min_cell_frames': 2,
    'match_distance': 1.0,
    'merge_distance': 1.0,
    'min_observations': 5,
    'require_known_map': True,
    'map_topic': '/map',
    'publish_debug_image': True,
    # Cámara térmica (fase 2): sin camera_info, como el MLX90640 real; intrínsecos ideales.
    'thermal_topic': '/thermal/image',
    'thermal_horizontal_fov': 0.96,
    'thermal_processing_rate': 4.0,
    'thermal_min_range': 0.15,
    'thermal_evidence_height': 0.1,
    'thermal_ray_beyond': 0.4,
    'thermal_bearing_cell': 0.15,
    'thermal_bearing_bins': 12,
    'thermal_min_bearings': 3,
    'map_occupied_threshold': 65,
    'log_dir': '',
    'log_prefix': 'victims',
}
CSV_HEADER = ['event', 'sim_time', 'victim_id', 'x', 'y', 'z', 'observations', 'confidence',
              'sources']
#: Color (RGB) de la máscara de cada detector en la imagen de depuración.
DEBUG_COLORS = {'color_shape': (0, 127, 127), 'human_shape': (127, 0, 127)}


def quaternion_matrix(q):
    """Matriz de rotación 3×3 de un cuaternión (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


#: Nombre corto de cada detector en la etiqueta de los marcadores.
SOURCE_LABELS = {'color_shape': 'color', 'human_shape': 'forma', 'thermal': 'calor'}


def _matrix(tf):
    """Matriz homogénea 4×4 de un TransformStamped."""
    m = np.eye(4)
    m[:3, :3] = quaternion_matrix(tf.transform.rotation)
    m[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y,
                tf.transform.translation.z]
    return m


class _Retracted:
    """Fila de log de una víctima retirada (solo el número)."""

    def __init__(self, victim_id):
        self.victim_id, self.position = victim_id, (float('nan'),) * 3
        self.observations, self.confidence, self.sources = 0, 0.0, []


class VictimDetector(Node):
    """Ejecuta los detectores de signos de vida y confirma víctimas en el mapa."""

    def __init__(self):
        super().__init__('victim_detector',
                         automatically_declare_parameters_from_overrides=True)
        self.p = {name: self._param(name, default) for name, default in PARAMS.items()}
        self.detectors = [create_detector(name, self._prefixed(name))
                          for name in self.p['detectors']]
        self.rgbd_detectors = [d for d in self.detectors if d.sensor == 'rgbd']
        self.thermal_detectors = [d for d in self.detectors if d.sensor == 'thermal']
        self.bearing_evidence = {
            d.name: BearingEvidence(self.p['thermal_bearing_cell'],
                                    self.p['thermal_bearing_bins'],
                                    self.p['thermal_min_bearings'])
            for d in self.thermal_detectors}
        self.trackers = {
            det.name: VictimTracker(
                self.p['min_observations'], cell_size=self.p['evidence_cell'],
                join_distance=self.p['join_distance'], min_cell_frames=self.p['min_cell_frames'],
                match_distance=self.p['match_distance'], shape=det.group_shape())
            for det in self.detectors}
        self.registry = VictimRegistry(self.p['merge_distance'], self.p['match_distance'])
        self.sensor_to_base = {}
        self.map_msg = None
        self.map_grid = None
        self.thermal_frames = 0
        self.thermal_last = None
        self.rays_cast = 0
        self.rays_hit = 0           # celdas de evidencia trianguladas (acumulado)
        self.points_outside_map = 0
        self.bridge = CvBridge()
        self.camera_k = None
        self.last_processed = None
        self.frames = 0
        self.pairs = 0
        self.tf_failures = 0
        self.detections = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        if self.rgbd_detectors:
            self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info,
                                     qos_profile_sensor_data)
            rgb_sub = message_filters.Subscriber(self, Image, self.p['rgb_topic'],
                                                 qos_profile=qos_profile_sensor_data)
            depth_sub = message_filters.Subscriber(self, Image, self.p['depth_topic'],
                                                   qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub], queue_size=5, slop=self.p['sync_slop'])
            self.sync.registerCallback(self._on_images)
        if self.thermal_detectors:
            self.create_subscription(Image, self.p['thermal_topic'], self._on_thermal,
                                     qos_profile_sensor_data)
        if self.p['require_known_map'] or self.thermal_detectors:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, self.p['map_topic'], self._on_map, latched)

        self.marker_pub = self.create_publisher(MarkerArray, '/victims/markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/victims/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/victims/debug_image', 2)
        self.thermal_debug_pub = self.create_publisher(Image, '/thermal/debug_image', 2)
        # Confirmaciones de cada detector por separado (métricas por modalidad).
        self.source_pubs = {det.name: self.create_publisher(
            PoseArray, f'/victims/detections/{det.name}', 10) for det in self.detectors}
        self.log_path, self._log = self._open_log()
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'Detectores: {self.p["detectors"]}; confirmación tras '
            f'{self.p["min_observations"]} observaciones; log: {self.log_path or "desactivado"}')

    # ------------------------------------------------------------------ parámetros
    def _param(self, name, default):
        return self.get_parameter_or(name, Parameter(name, value=default)).value

    def _prefixed(self, prefix):
        """Parámetros `<prefix>.<clave>` como diccionario (configuración de un detector)."""
        return {k: v.value for k, v in self.get_parameters_by_prefix(prefix).items()}

    # ------------------------------------------------------------------ log
    def _open_log(self):
        log_dir = self.p['log_dir']
        if not log_dir:
            return None, None
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.abspath(os.path.join(log_dir, f'{self.p["log_prefix"]}_{stamp}.csv'))
        stream = open(path, 'w', newline='', encoding='utf-8')
        writer = csv.writer(stream)
        writer.writerow(CSV_HEADER)
        stream.flush()
        return path, (stream, writer)

    def _log_victim(self, event, victim, sim_time):
        if self._log is None:
            return
        stream, writer = self._log
        x, y, z = victim.position
        writer.writerow([event, f'{sim_time:.2f}', victim.victim_id, f'{x:.3f}', f'{y:.3f}',
                         f'{z:.3f}', victim.observations, f'{victim.confidence:.2f}',
                         '+'.join(victim.sources)])
        stream.flush()

    # ------------------------------------------------------------------ procesamiento
    def _on_info(self, msg):
        self.camera_k = list(msg.k)

    def _camera_transform(self, frame_id, stamp):
        """
        TF global ← cámara en el instante de la imagen.

        La imagen suele llegar unos milisegundos antes que la odometría de ese instante; en ese
        caso se usa la TF más reciente si no difiere más de tf_tolerance de la imagen.
        """
        target = self.p['global_frame']
        try:
            return self.tf_buffer.lookup_transform(target, frame_id, stamp)
        except tf2_ros.ExtrapolationException:
            pass
        except tf2_ros.TransformException as error:
            self._tf_failed(error)
            return None
        try:
            tf = self.tf_buffer.lookup_transform(target, frame_id, Time())
        except tf2_ros.TransformException as error:
            self._tf_failed(error)
            return None
        age = abs((stamp - Time.from_msg(tf.header.stamp)).nanoseconds) * 1e-9
        if age > self.p['tf_tolerance']:
            self._tf_failed(f'TF desfasada {age:.3f} s respecto a la imagen')
            return None
        return tf

    def _base_transform(self, frame_id):
        """Matriz 4×4 base ← sensor (fija: se consulta una vez por frame)."""
        if frame_id not in self.sensor_to_base:
            try:
                tf = self.tf_buffer.lookup_transform(self.p['base_frame'], frame_id, Time())
            except tf2_ros.TransformException:
                return None
            self.sensor_to_base[frame_id] = _matrix(tf)
        return self.sensor_to_base[frame_id]

    def _on_map(self, msg):
        self.map_msg = msg
        self.map_grid = np.asarray(msg.data, dtype=np.int8)

    def _tf_failed(self, error):
        self.tf_failures += 1
        if self.tf_failures == 1:
            self.get_logger().warn(f'Sin TF para la imagen: {error}')

    def _report(self):
        """Resumen periódico del procesamiento (diagnóstico)."""
        self.get_logger().info(
            f'Imágenes: {self.pairs} pares recibidos, {self.frames} procesados, '
            f'{self.tf_failures} sin TF; {self.detections} detecciones '
            f'({self.points_outside_map} puntos fuera del mapa conocido); '
            + (f'térmica: {self.thermal_frames} imágenes, {self.rays_cast} rayos, '
               f'{self.rays_hit} celdas trianguladas; ' if self.thermal_detectors else '')
            + f'{len(self.registry.victims)} víctimas confirmadas; por detector: ' + ', '.join(
                f'{name} {len(t.confirmed())} confirmadas/{len(t.tentative())} tentativas'
                for name, t in self.trackers.items()))

    def _on_images(self, rgb_msg, depth_msg):
        self.pairs += 1
        if self.camera_k is None:
            return
        stamp = Time.from_msg(rgb_msg.header.stamp)
        sec = stamp.nanoseconds * 1e-9
        if self.last_processed is not None and \
                sec - self.last_processed < 1.0 / self.p['processing_rate']:
            return
        tf = self._camera_transform(rgb_msg.header.frame_id, stamp)
        if tf is None:
            return
        self.last_processed = sec
        self.frames += 1
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        obs = Observation(rgb=rgb, depth=depth, k=self.camera_k, stamp_sec=sec,
                          frame_id=rgb_msg.header.frame_id,
                          sensor_to_base=self._base_transform(rgb_msg.header.frame_id))
        world = _matrix(tf)
        all_detections = []
        for detector in self.rgbd_detectors:
            detections = detector.detect(obs)
            self.detections += len(detections)
            all_detections.extend(detections)
            sensor_points = [d.points if d.points is not None else np.array([d.point])
                             for d in detections]
            points = (np.vstack(sensor_points) @ world[:3, :3].T + world[:3, 3]) \
                if sensor_points else np.zeros((0, 3))
            self.trackers[detector.name].update(self._in_known_map(points), sec)
        self._update_registry(sec, rgb_msg.header.stamp)
        if self.p['publish_debug_image'] and self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(rgb, all_detections, rgb_msg.header)

    def _on_thermal(self, msg):
        """Imagen térmica: manchas calientes → rayos → primera celda ocupada del /map."""
        stamp = Time.from_msg(msg.header.stamp)
        sec = stamp.nanoseconds * 1e-9
        if self.thermal_last is not None and \
                sec - self.thermal_last < 1.0 / self.p['thermal_processing_rate'] - 1e-3:
            return
        if msg.encoding != '32FC1':
            self._tf_failed(f'imagen térmica con codificación {msg.encoding} (se espera 32FC1)')
            return
        tf = self._camera_transform(msg.header.frame_id, stamp)
        if tf is None:
            return
        self.thermal_last = sec
        self.thermal_frames += 1
        temperature = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        obs = ThermalObservation(
            temperature=temperature, stamp_sec=sec, frame_id=msg.header.frame_id,
            k=intrinsics_from_fov(msg.width, msg.height, self.p['thermal_horizontal_fov']),
            sensor_to_base=self._base_transform(msg.header.frame_id))
        world = _matrix(tf)
        all_detections = []
        for detector in self.thermal_detectors:
            detections = detector.detect(obs)
            self.detections += len(detections)
            all_detections.extend(detections)
            points = self._triangulate(detector.name, detections, world)
            self.trackers[detector.name].update(self._in_known_map(points), sec)
        self._update_registry(sec, msg.header.stamp)
        if self.p['publish_debug_image'] and \
                self.thermal_debug_pub.get_subscription_count() > 0:
            self._publish_thermal_debug(temperature, all_detections, msg.header)

    def _triangulate(self, name, detections, world):
        """
        Puntos de evidencia (frame global) de detecciones con rayos y sin distancia.

        Cada rayo se recorta con el /map del robot (hasta el primer obstáculo más
        thermal_ray_beyond, la primera celda desconocida o su distancia máxima) y se suma a la
        rejilla de intersección; son evidencia las celdas que este rayo toca y que ya se han
        cruzado desde thermal_min_bearings direcciones distintas.
        """
        if not detections or self.map_msg is None:
            return np.zeros((0, 3))
        rays = np.vstack([d.rays for d in detections]) @ world[:3, :3].T
        bounds = np.concatenate([d.ray_max_range for d in detections])
        info = self.map_msg.info
        origin = world[:3, 3]
        lengths = ray_extents(self.map_grid, info.origin.position.x, info.origin.position.y,
                              info.resolution, info.width, info.height, origin[:2], rays[:, :2],
                              bounds, self.p['thermal_min_range'], self.p['thermal_ray_beyond'],
                              self.p['map_occupied_threshold'])
        self.rays_cast += len(lengths)
        xy = self.bearing_evidence[name].add_rays(origin[:2], rays[:, :2], lengths,
                                                  self.p['thermal_min_range'])
        self.rays_hit += len(xy)
        return np.column_stack([xy, np.full(len(xy), self.p['thermal_evidence_height'])])

    def _update_registry(self, sec, stamp):
        """Fusiona las confirmaciones de todos los detectores, registra eventos y publica."""
        newly, retracted = self.registry.update(
            {name: t.confirmed() for name, t in self.trackers.items()},
            self.p['min_observations'])
        for victim in newly:
            x, y, _ = victim.position
            self.get_logger().info(
                f'Víctima {victim.victim_id} confirmada en ({x:.2f}, {y:.2f}) por '
                f'{"+".join(victim.sources)} tras {victim.observations} observaciones')
            self._log_victim('confirmed', victim, sec)
        for vid in retracted:
            self.get_logger().info(f'Víctima {vid} retirada: ya no la confirma ningún detector '
                                   '(su grupo dejó de cumplir la forma o se unió a otro)')
            self._log_victim('retracted', _Retracted(vid), sec)
        self._publish(stamp)

    def _in_known_map(self, points):
        """Puntos que caen en celdas conocidas del /map (sin mapa aún: se descartan todos)."""
        if not self.p['require_known_map'] or len(points) == 0:
            return points
        if self.map_msg is None:
            return points[:0]
        info = self.map_msg.info
        keep = known_cells_mask(points, self.map_grid, info.origin.position.x,
                                info.origin.position.y, info.resolution, info.width,
                                info.height)
        self.points_outside_map += int((~keep).sum())
        return points[keep]

    # ------------------------------------------------------------------ salidas
    def _publish(self, stamp):
        frame = self.p['global_frame']
        markers = [Marker(action=Marker.DELETEALL)]
        poses = PoseArray()
        poses.header.frame_id, poses.header.stamp = frame, stamp

        def marker(ns, mid, mtype, xyz, scale, color):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            return m

        for victim in self.registry.victims:
            x, y, _ = victim.position
            markers.append(marker('victims', victim.victim_id, Marker.SPHERE, (x, y, 0.3), 0.45,
                                  (1.0, 0.1, 0.1, 0.8)))
            label = marker('labels', victim.victim_id, Marker.TEXT_VIEW_FACING, (x, y, 0.9),
                           0.3, (1.0, 1.0, 1.0, 1.0))
            label.text = (f'Víctima {victim.victim_id} ({victim.confidence * 100:.0f} %, '
                          f'{"+".join(SOURCE_LABELS.get(s, s) for s in victim.sources)})')
            markers.append(label)
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            pose.orientation.w = 1.0
            poses.poses.append(pose)
        mid = 0
        for tracker in self.trackers.values():
            for track in tracker.tentative():
                x, y, _ = track.position
                mid += 1
                markers.append(marker('tentative', mid, Marker.SPHERE, (x, y, 0.2), 0.2,
                                      (0.7, 0.7, 0.7, 0.6)))
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.pose_pub.publish(poses)
        for name, pub in self.source_pubs.items():
            per_source = PoseArray()
            per_source.header = poses.header
            for track in self.trackers[name].confirmed():
                pose = Pose()
                pose.position.x, pose.position.y = track.position[0], track.position[1]
                pose.orientation.w = 1.0
                per_source.poses.append(pose)
            pub.publish(per_source)

    def _publish_debug(self, rgb, detections, header):
        overlay = rgb.copy()
        for detector in self.rgbd_detectors:
            mask = detector.debug_mask()
            if mask is not None:
                tint = DEBUG_COLORS.get(detector.name, (127, 127, 0))
                overlay[mask > 0] = (0.5 * overlay[mask > 0] + tint).astype(np.uint8)
        for det in detections:
            u, v = int(det.pixel[0]), int(det.pixel[1])
            cv2.drawMarker(overlay, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(overlay, f'{det.point[2]:.1f} m', (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding='rgb8')
        msg.header = header
        self.debug_pub.publish(msg)

    def _publish_thermal_debug(self, temperature, detections, header, scale=10):
        """Imagen térmica ampliada en falso color, con el contorno de las manchas calientes."""
        temp = np.nan_to_num(temperature, nan=0.0)
        lo, hi = 15.0, 40.0
        norm = np.clip((temp - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
        big = cv2.resize(norm, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        color = cv2.cvtColor(cv2.applyColorMap(big, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)
        for detector in self.thermal_detectors:
            mask = detector.debug_mask()
            if mask is not None:
                mask_big = cv2.resize(mask, None, fx=scale, fy=scale,
                                      interpolation=cv2.INTER_NEAREST)
                contours, _ = cv2.findContours(mask_big, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(color, contours, -1, (0, 255, 0), 2)
        for det in detections:
            u, v = int(det.pixel[0] * scale), int(det.pixel[1] * scale)
            cv2.putText(color, f'{det.extra["temperature"]:.1f} C', (u + 6, v - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        msg = self.bridge.cv2_to_imgmsg(color, encoding='rgb8')
        msg.header = header
        self.thermal_debug_pub.publish(msg)

    def destroy_node(self):
        """Escribe el resumen final de víctimas en el log antes de cerrar."""
        sim_time = max(self.last_processed or 0.0, self.thermal_last or 0.0)
        for victim in self.registry.victims:
            self._log_victim('final', victim, sim_time)
        if self._log is not None:
            self._log[0].close()
        # Al cerrar con Ctrl+C el contexto de ROS puede estar ya invalidado: el resumen va también
        # a la salida estándar para no perderlo.
        print(f'[victim_detector] {len(self.registry.victims)} víctimas confirmadas; '
              f'log: {self.log_path or "desactivado"}', flush=True)
        super().destroy_node()


def main():
    rclpy.init()
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Carrera de rclpy al cerrar con Ctrl+C (lee un mensaje con el contexto ya cerrado);
        # cualquier otro RuntimeError se propaga.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
