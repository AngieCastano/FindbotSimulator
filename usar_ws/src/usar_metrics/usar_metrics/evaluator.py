"""
Nodo evaluator: calcula las métricas de la misión contra el ground truth.

Es el ÚNICO nodo que lee ground_truth.yaml. No publica nada que use el robot: solo escucha
/map, /odom, /victims/detections, /exploration/status, el TF map → base_footprint y, en
simulación, la pose real del robot (/sim/ground_truth/odom), y escribe los resultados en disco.

Métricas: cobertura C%, tiempo de exploración, distancia recorrida, víctimas detectadas, tasa
de detección, falsos positivos, error de localización de las víctimas y error de localización
del robot (pose de SLAM y de la odometría de ruedas frente a la pose real, cada
`pose_error_period` s: results/seed<N>_<fecha>_pose.csv y _pose_error.png). Cuando el
explorador termina (estado `finished:<motivo>`), espera `finish_delay` para recibir las últimas
detecciones, guarda results/seed<N>_<fecha>.{json,csv}, el mapa (pgm/yaml) y la imagen
comparativa, y termina (el launch de la misión se cierra entonces).
"""

from collections import deque
from datetime import datetime
import math
import os
from typing import Dict, List, Optional

from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
from usar_msgs.msg import VictimDetectionArray
from usar_worldgen.ground_truth import explorable_from_ground_truth, load_ground_truth

from . import report
from .metrics_core import (associate_victims, coverage, explorable_centers, GridGeometry,
                           localization_summary, map_to_world, nearest_in_time, PathIntegrator,
                           Pose2D, pose_error, pose_to_world)

PARAMS = {
    'ground_truth_file': '',
    'results_dir': 'results',
    'result_prefix': '',
    'association_threshold': 1.0,
    'sample_period': 10.0,
    'free_threshold': 25,
    'occupied_threshold': 65,
    'odom_min_step': 0.001,
    'trajectory_step': 0.05,
    'finish_delay': 3.0,
    'max_mission_time': 1200.0,
    'map_topic': '/map',
    'odom_topic': '/odom',
    'detections_topic': '/victims/detections',
    # Detecciones de cada detector por separado (/victims/detections/<modalidad>): métricas por
    # modalidad sensorial. Solo se informan las modalidades que publicaron algo.
    'modalities': ['color_shape', 'human_shape', 'thermal', 'co2', 'sound', 'radar'],
    'status_topic': '/exploration/status',
    'image_dpi': 150,
    # Error de localización del robot (solo simulación: sin pose real no se calcula).
    'true_pose_topic': '',
    'map_frame': 'map',
    'robot_frame': 'base_footprint',
    'pose_error_period': 1.0,
    'pose_sync_tolerance': 0.1,
}

WAITING_STATES = ('', 'waiting')
REPORTED_PARAMS = ('association_threshold', 'free_threshold', 'occupied_threshold',
                   'sample_period')


class Evaluator(Node):
    """Acumula los datos de la misión y escribe los resultados al terminar."""

    def __init__(self):
        super().__init__('evaluator')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMS}

        gt_path = os.path.abspath(os.path.expanduser(self.p['ground_truth_file']))
        if not os.path.isfile(gt_path):
            raise FileNotFoundError(f'No existe el ground truth: {gt_path!r}')
        self.gt = load_ground_truth(gt_path)
        spec, mask = explorable_from_ground_truth(self.gt)
        self.explorable = explorable_centers(
            GridGeometry(spec.origin_x, spec.origin_y, spec.resolution, spec.cols, spec.rows),
            mask)
        self.explorable_area = float(mask.sum()) * spec.resolution ** 2
        spawn = self.gt['robot_spawn']
        self.spawn = Pose2D(float(spawn['x']), float(spawn['y']), float(spawn['yaw']))

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = self.p['result_prefix'] or f"seed{self.gt['seed']}"
        self.results_dir = os.path.abspath(os.path.expanduser(self.p['results_dir']))
        os.makedirs(self.results_dir, exist_ok=True)
        self.base = os.path.join(self.results_dir, f'{prefix}_{stamp}')

        self.map_msg: Optional[OccupancyGrid] = None
        self.estimates: List = []
        self.modality_estimates: Dict[str, List] = {}
        self.status = ''
        self.path = PathIntegrator(self.p['odom_min_step'])
        self.trajectory: List = []
        self.first_sim_time: Optional[float] = None
        self.start_time: Optional[float] = None     # inicio de la exploración (tiempo simulado)
        self.end_time: Optional[float] = None
        self.end_reason: Optional[str] = None
        self.samples: List[Dict] = []
        self.done = False
        self._finish_timer = None
        self._last_now = 0.0
        # Error de localización: búferes (tiempo, pose) de la pose real y de la odometría.
        self.true_poses = deque(maxlen=200)
        self.odom_poses = deque(maxlen=200)
        self.pose_rows: List[Dict] = []
        self._last_pose_stamp = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.p['map_topic'],
                                 lambda m: setattr(self, 'map_msg', m), latched)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, 50)
        self.create_subscription(VictimDetectionArray, self.p['detections_topic'],
                                 self._on_detections, 10)
        for name in self.p['modalities']:
            self.create_subscription(
                VictimDetectionArray, f"{self.p['detections_topic']}/{name}",
                lambda msg, name=name: self.modality_estimates.__setitem__(
                    name, [(v.position.x, v.position.y) for v in msg.victims]), 10)
        self.create_subscription(String, self.p['status_topic'], self._on_status, latched)
        self.create_timer(float(self.p['sample_period']), self._sample)
        if self.p['true_pose_topic']:
            self.create_subscription(Odometry, self.p['true_pose_topic'], self._on_true_pose, 50)
            self.create_timer(float(self.p['pose_error_period']), self._sample_pose_error)
        self.get_logger().info(
            f"Evaluando la semilla {self.gt['seed']}: {len(self.gt['victims'])} víctimas, área "
            f'explorable {self.explorable_area:.1f} m²; resultados en {self.base}.*')

    # ------------------------------------------------------------------ entradas
    def _now(self) -> float:
        """Tiempo simulado actual (el último conocido si el contexto de ROS ya se cerró)."""
        try:
            self._last_now = self.get_clock().now().nanoseconds * 1e-9
        except Exception:  # noqa: B902  (Ctrl+C: el reloj puede no estar disponible)
            pass
        return self._last_now

    @staticmethod
    def _pose_of(msg: Odometry):
        """(tiempo, Pose2D) de un mensaje de odometría."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return t, Pose2D(p.x, p.y, yaw)

    def _on_true_pose(self, msg: Odometry):
        self.true_poses.append(self._pose_of(msg))

    def _sample_pose_error(self):
        """Compara la pose de SLAM (TF map → robot) y la de /odom con la pose real."""
        if self.end_time is not None or not self.true_poses:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.p['map_frame'], self.p['robot_frame'],
                                                 Time())
        except tf2_ros.TransformException:
            return
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if t == self._last_pose_stamp:
            return
        self._last_pose_stamp = t
        tol = float(self.p['pose_sync_tolerance'])
        truth = nearest_in_time(self.true_poses, t, tol)
        if truth is None:
            return
        q = tf.transform.rotation
        slam = pose_to_world(Pose2D(
            tf.transform.translation.x, tf.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))),
            self.spawn)
        slam_err, slam_yaw = pose_error(slam, truth)
        row = {'sim_time': round(t, 3),
               'true_x': round(truth.x, 4), 'true_y': round(truth.y, 4),
               'true_yaw': round(truth.yaw, 4),
               'slam_x': round(slam.x, 4), 'slam_y': round(slam.y, 4),
               'slam_yaw': round(slam.yaw, 4),
               'slam_error_m': round(slam_err, 4),
               'slam_yaw_error_deg': round(math.degrees(slam_yaw), 3)}
        odom = nearest_in_time(self.odom_poses, t, tol)
        if odom is not None:
            odom = pose_to_world(odom, self.spawn)
            odom_err, odom_yaw = pose_error(odom, truth)
            row.update({'odom_x': round(odom.x, 4), 'odom_y': round(odom.y, 4),
                        'odom_yaw': round(odom.yaw, 4), 'odom_error_m': round(odom_err, 4),
                        'odom_yaw_error_deg': round(math.degrees(odom_yaw), 3)})
        self.pose_rows.append(row)

    def _on_odom(self, msg: Odometry):
        if self.end_time is not None:
            return
        self.odom_poses.append(self._pose_of(msg))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.path.add(x, y)
        if (not self.trajectory or math.hypot(x - self.trajectory[-1][0],
                                              y - self.trajectory[-1][1])
                >= self.p['trajectory_step']):
            self.trajectory.append((x, y))

    def _on_detections(self, msg: VictimDetectionArray):
        self.estimates = [(v.position.x, v.position.y) for v in msg.victims]

    def _on_status(self, msg: String):
        self.status = msg.data
        self._check_start()
        if self.status.startswith('finished') and self.end_time is None:
            self._finish(self.status.split(':', 1)[-1] if ':' in self.status else 'finished')

    def _check_start(self):
        """Fija el inicio de la exploración: primer estado activo con el reloj ya en marcha."""
        now = self._now()
        if self.start_time is None and self.status not in WAITING_STATES and now > 0.0:
            self.start_time = now
            self.get_logger().info(f'Inicio de la exploración (t = {now:.1f} s)')

    # ------------------------------------------------------------------ métricas
    def _exploration_time(self, now: Optional[float] = None) -> float:
        if self.start_time is None:
            return 0.0
        end = self.end_time if self.end_time is not None else (now or self._now())
        return max(0.0, end - self.start_time)

    def _map_geometry(self) -> GridGeometry:
        info = self.map_msg.info
        return GridGeometry(info.origin.position.x, info.origin.position.y, info.resolution,
                            info.width, info.height)

    def _coverage(self) -> float:
        if self.map_msg is None:
            return 0.0
        pct, _ = coverage(np.asarray(self.map_msg.data), self._map_geometry(),
                          self.explorable, self.spawn, int(self.p['free_threshold']))
        return pct

    def _victims(self, estimates=None):
        estimates = self.estimates if estimates is None else estimates
        world = map_to_world(np.asarray(estimates, dtype=float).reshape(-1, 2), self.spawn)
        return associate_victims([tuple(p) for p in world], self.gt['victims'],
                                 float(self.p['association_threshold']))

    def _sample(self):
        """Muestra periódica de la evolución; comprueba el tiempo máximo de la misión."""
        now = self._now()
        self._check_start()
        if self.first_sim_time is None:
            if now <= 0.0:
                return
            self.first_sim_time = now
        victims = self._victims()
        row = {
            'sim_time': round(now, 2),
            'exploration_time': round(self._exploration_time(now), 2),
            'coverage_pct': round(self._coverage(), 2),
            'distance_m': round(self.path.distance, 3),
            'victims_estimated': victims.detected + len(victims.false_positives),
            'victims_detected': victims.detected,
            'false_positives': len(victims.false_positives),
            'status': self.status,
        }
        if self.end_time is None:
            self.samples.append(row)
        if (self.end_time is None
                and now - self.first_sim_time > float(self.p['max_mission_time'])):
            self.get_logger().warn('Tiempo máximo de la misión agotado sin fin del explorador')
            self._finish('evaluator_timeout')

    # ------------------------------------------------------------------ fin
    def _finish(self, reason: str):
        """Marca el fin; espera finish_delay (tiempo simulado) y escribe los resultados."""
        self.end_time = self._now()
        self.end_reason = reason
        self.get_logger().info(f'Fin de la exploración ({reason}); escribiendo resultados en '
                               f"{self.p['finish_delay']:.0f} s")
        self._finish_timer = self.create_timer(float(self.p['finish_delay']), self._write_once)

    def _write_once(self):
        if self._finish_timer is not None:
            self._finish_timer.cancel()
            self._finish_timer = None
        if not self.done:
            self.write_results()
            self.done = True

    def write_results(self) -> Optional[Dict]:
        """Calcula el resumen final y escribe todos los archivos."""
        if self.end_time is None:
            self.end_time = self._now()
            self.end_reason = self.end_reason or 'interrupted'
        victims = self._victims()
        cov = self._coverage()
        summary = {
            'seed': self.gt['seed'],
            'date': datetime.now().isoformat(timespec='seconds'),
            'end_reason': self.end_reason,
            'exploration_time_s': round(self._exploration_time(), 2),
            'coverage_pct': round(cov, 2),
            'explored_area_m2': round(cov / 100.0 * self.explorable_area, 3),
            'explorable_area_m2': round(self.explorable_area, 3),
            'distance_m': round(self.path.distance, 3),
            'victims': victims.summary(),
            'victim_details': report.victim_rows(victims),
            'false_positive_positions': [[round(x, 4), round(y, 4)]
                                         for x, y in victims.false_positives],
            # Lo que habría encontrado cada detector por sí solo (sin la fusión).
            'by_modality': {name: self._victims(est).summary()
                            for name, est in sorted(self.modality_estimates.items())},
            # Error de localización del robot frente a la pose real (None sin pose real).
            'robot_localization': localization_summary(self.pose_rows),
            'world_params': self.gt.get('params', {}),
            'evaluator_params': {k: self.p[k] for k in REPORTED_PARAMS},
            'files': {},
        }
        self.samples.append({
            'sim_time': round(self.end_time, 2),
            'exploration_time': summary['exploration_time_s'],
            'coverage_pct': summary['coverage_pct'], 'distance_m': summary['distance_m'],
            'victims_estimated': summary['victims']['n_estimated'],
            'victims_detected': victims.detected,
            'false_positives': len(victims.false_positives), 'status': self.status,
        })
        files = summary['files']
        files['timeseries_csv'] = self.base + '.csv'
        report.write_timeseries(files['timeseries_csv'], self.samples)
        if self.map_msg is not None:
            geometry = self._map_geometry()
            data = np.asarray(self.map_msg.data)
            files['map_pgm'], files['map_yaml'] = report.write_map(
                self.base + '_map', data, geometry, int(self.p['free_threshold']),
                int(self.p['occupied_threshold']))
            files['comparison_png'] = self.base + '_comparison.png'
            trajectory = map_to_world(np.asarray(self.trajectory).reshape(-1, 2), self.spawn)
            report.render_comparison(files['comparison_png'], self.gt, data, geometry,
                                     self.spawn, victims, trajectory, summary,
                                     int(self.p['free_threshold']),
                                     int(self.p['occupied_threshold']),
                                     int(self.p['image_dpi']))
        if self.pose_rows:
            files['pose_csv'] = self.base + '_pose.csv'
            report.write_pose_errors(files['pose_csv'], self.pose_rows)
            files['pose_error_png'] = self.base + '_pose_error.png'
            report.render_pose_errors(
                files['pose_error_png'], self.pose_rows,
                f"Semilla {summary['seed']}: error de localización del robot",
                int(self.p['image_dpi']))
        files['summary_json'] = self.base + '.json'
        report.write_json(files['summary_json'], summary)

        vs = summary['victims']
        err = vs['localization_error_mean_m']
        print(f"[evaluator] Semilla {summary['seed']} ({summary['end_reason']}): cobertura "
              f"{summary['coverage_pct']:.1f} %, tiempo {summary['exploration_time_s']:.0f} s, "
              f"recorrido {summary['distance_m']:.1f} m, víctimas {vs['detected']}/"
              f"{vs['n_victims']}, falsos positivos {vs['false_positives']}, error medio "
              f"{'-' if err is None else f'{err:.2f} m'}", flush=True)
        for name, mod in summary['by_modality'].items():
            print(f"[evaluator]   {name}: víctimas {mod['detected']}/{mod['n_victims']}, "
                  f"falsos positivos {mod['false_positives']}", flush=True)
        loc = summary['robot_localization']
        if loc:
            nan = float('nan')
            print('[evaluator]   localización del robot: SLAM media '
                  f"{loc['slam_error_m_mean']:.2f} m / máx. {loc['slam_error_m_max']:.2f} m; "
                  f"odometría media {loc.get('odom_error_m_mean', nan):.2f} m / máx. "
                  f"{loc.get('odom_error_m_max', nan):.2f} m", flush=True)
        print(f"[evaluator] Resultados: {files['summary_json']}", flush=True)
        return summary


def main(args=None):
    """Punto de entrada: termina cuando escribe los resultados (o con Ctrl+C)."""
    rclpy.init(args=args)
    node = Evaluator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Misión interrumpida: se guardan los resultados parciales.
        if not node.done:
            node.end_reason = 'interrupted'
            node.write_results()
            node.done = True
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
