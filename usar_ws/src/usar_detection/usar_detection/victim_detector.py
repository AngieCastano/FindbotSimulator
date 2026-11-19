"""
Nodo victim_detector: detecta víctimas con la cámara RGB-D y las ubica en el mapa.

Entradas (solo sensores y estimación propia del robot):
  /camera/image, /camera/depth_image, /camera/camera_info
  TF map → camera_optical_frame (SLAM + modelo del robot)
Salidas:
  /victims/markers     (visualization_msgs/MarkerArray) víctimas confirmadas y tentativas
  /victims/detections  (geometry_msgs/PoseArray, frame map) víctimas confirmadas
  /victims/debug_image (sensor_msgs/Image) imagen con la segmentación superpuesta
  log CSV en log_dir   (tiempo, víctima, posición estimada, observaciones, confianza)

Los detectores de signos de vida se eligen con el parámetro `detectors`; cada uno lee sus
parámetros con el prefijo de su nombre (p. ej. color_shape.hsv_lower).
Regla de diseño: este nodo NUNCA lee ground_truth.yaml ni poses de Gazebo.
"""

import csv
from datetime import datetime
import os

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import message_filters
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
import tf2_ros
from usar_detection import color_detector  # noqa: F401  (registra ColorShapeDetector)
from usar_detection.sign_of_life import create_detector, Observation
from usar_detection.tracker import VictimTracker
from visualization_msgs.msg import Marker, MarkerArray

PARAMS = {
    'detectors': ['color_shape'],
    'rgb_topic': '/camera/image',
    'depth_topic': '/camera/depth_image',
    'camera_info_topic': '/camera/camera_info',
    'global_frame': 'map',
    'processing_rate': 5.0,
    'sync_slop': 0.05,
    'tf_tolerance': 0.1,
    'evidence_cell': 0.1,
    'join_distance': 0.2,
    'min_cell_frames': 2,
    'match_distance': 1.0,
    'min_observations': 5,
    'publish_debug_image': True,
    'log_dir': '',
    'log_prefix': 'victims',
}
CSV_HEADER = ['event', 'sim_time', 'victim_id', 'x', 'y', 'z', 'observations', 'confidence']


def quaternion_matrix(q):
    """Matriz de rotación 3×3 de un cuaternión (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class VictimDetector(Node):
    """Ejecuta los detectores de signos de vida y confirma víctimas en el mapa."""

    def __init__(self):
        super().__init__('victim_detector',
                         automatically_declare_parameters_from_overrides=True)
        self.p = {name: self._param(name, default) for name, default in PARAMS.items()}
        self.detectors = [create_detector(name, self._prefixed(name))
                          for name in self.p['detectors']]
        self.tracker = VictimTracker(
            self.p['min_observations'], cell_size=self.p['evidence_cell'],
            join_distance=self.p['join_distance'], min_cell_frames=self.p['min_cell_frames'],
            match_distance=self.p['match_distance'])
        self.bridge = CvBridge()
        self.camera_k = None
        self.last_processed = None
        self.frames = 0
        self.pairs = 0
        self.tf_failures = 0
        self.detections = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info,
                                 qos_profile_sensor_data)
        rgb_sub = message_filters.Subscriber(self, Image, self.p['rgb_topic'],
                                             qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, self.p['depth_topic'],
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=self.p['sync_slop'])
        self.sync.registerCallback(self._on_images)

        self.marker_pub = self.create_publisher(MarkerArray, '/victims/markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/victims/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/victims/debug_image', 2)
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

    def _log_track(self, event, track, sim_time):
        if self._log is None:
            return
        stream, writer = self._log
        x, y, z = track.position
        writer.writerow([event, f'{sim_time:.2f}', track.victim_id, f'{x:.3f}', f'{y:.3f}',
                         f'{z:.3f}', track.observations,
                         f'{track.confidence(self.p["min_observations"]):.2f}'])
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

    def _tf_failed(self, error):
        self.tf_failures += 1
        if self.tf_failures == 1:
            self.get_logger().warn(f'Sin TF para la imagen: {error}')

    def _report(self):
        """Resumen periódico del procesamiento (diagnóstico)."""
        self.get_logger().info(
            f'Imágenes: {self.pairs} pares recibidos, {self.frames} procesados, '
            f'{self.tf_failures} sin TF; {self.detections} detecciones; '
            f'{len(self.tracker.confirmed())} víctimas confirmadas, '
            f'{len(self.tracker.tentative())} tentativas')

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
                          frame_id=rgb_msg.header.frame_id)
        rot = quaternion_matrix(tf.transform.rotation)
        trans = np.array([tf.transform.translation.x, tf.transform.translation.y,
                          tf.transform.translation.z])
        all_detections = []
        for detector in self.detectors:
            detections = detector.detect(obs)
            self.detections += len(detections)
            all_detections.extend(detections)
        sensor_points = [d.points if d.points is not None else np.array([d.point])
                         for d in all_detections]
        points = (np.vstack(sensor_points) @ rot.T + trans) if sensor_points \
            else np.zeros((0, 3))
        for track in self.tracker.update(points, sec):
            x, y, _ = track.position
            self.get_logger().info(
                f'Víctima {track.victim_id} confirmada en ({x:.2f}, {y:.2f}) '
                f'tras {track.observations} observaciones')
            self._log_track('confirmed', track, sec)
        self._publish(rgb_msg.header.stamp)
        if self.p['publish_debug_image'] and self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(rgb, all_detections, rgb_msg.header)

    # ------------------------------------------------------------------ salidas
    def _publish(self, stamp):
        frame = self.p['global_frame']
        markers = [Marker(action=Marker.DELETEALL)]
        poses = PoseArray()
        poses.header.frame_id, poses.header.stamp = frame, stamp
        n_min = self.p['min_observations']

        def marker(ns, mid, mtype, xyz, scale, color):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            return m

        for track in self.tracker.confirmed():
            x, y, _ = track.position
            markers.append(marker('victims', track.victim_id, Marker.SPHERE, (x, y, 0.3), 0.45,
                                  (1.0, 0.1, 0.1, 0.8)))
            label = marker('labels', track.victim_id, Marker.TEXT_VIEW_FACING, (x, y, 0.9),
                           0.3, (1.0, 1.0, 1.0, 1.0))
            label.text = (f'Víctima {track.victim_id} '
                          f'({track.confidence(n_min) * 100:.0f} %)')
            markers.append(label)
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            pose.orientation.w = 1.0
            poses.poses.append(pose)
        for track in self.tracker.tentative():
            x, y, _ = track.position
            markers.append(marker('tentative', track.track_id, Marker.SPHERE, (x, y, 0.2), 0.2,
                                  (0.7, 0.7, 0.7, 0.6)))
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.pose_pub.publish(poses)

    def _publish_debug(self, rgb, detections, header):
        overlay = rgb.copy()
        for detector in self.detectors:
            mask = detector.debug_mask()
            if mask is not None:
                overlay[mask > 0] = (0.5 * overlay[mask > 0] + [0, 127, 127]).astype(np.uint8)
        for det in detections:
            u, v = int(det.pixel[0]), int(det.pixel[1])
            cv2.drawMarker(overlay, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(overlay, f'{det.point[2]:.1f} m', (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding='rgb8')
        msg.header = header
        self.debug_pub.publish(msg)

    def destroy_node(self):
        """Escribe el resumen final de víctimas en el log antes de cerrar."""
        sim_time = self.last_processed or 0.0
        for track in self.tracker.confirmed():
            self._log_track('final', track, sim_time)
        if self._log is not None:
            self._log[0].close()
        # Al cerrar con Ctrl+C el contexto de ROS puede estar ya invalidado: el resumen va también
        # a la salida estándar para no perderlo.
        print(f'[victim_detector] {len(self.tracker.confirmed())} víctimas confirmadas; '
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
