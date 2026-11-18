#!/usr/bin/env python3
"""
Prueba de humo de SLAM + Nav2 (Fase 3).

Con nav.launch.py ya corriendo, verifica:
  - el árbol TF map → odom → base_footprint → sensores,
  - que slam_toolbox publica /map,
  - que la nube de profundidad está bien orientada (puntos delante del robot),
  - que NavigateToPose lleva al robot a una meta.

Uso (dentro del contenedor):
    python3 scripts/smoke_test_nav.py [--goal X Y] [--timeout S]
La meta por defecto (6.2, 3.2) es alcanzable en el mundo de la semilla 42.
"""

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

TF_CHAIN = [('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'base_link'),
            ('base_link', 'lidar_link'), ('base_link', 'camera_optical_frame')]


def _quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class NavSmokeTest(Node):
    """Nodo auxiliar que recoge mensajes y consulta TF."""

    def __init__(self):
        super().__init__('smoke_test_nav', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.map_msg = None
        self.cloud_msg = None
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(PointCloud2, '/camera/depth/points', self._on_cloud,
                                 qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_cloud(self, msg):
        self.cloud_msg = msg

    def spin_until(self, predicate, timeout):
        """Procesa callbacks hasta que predicate() sea cierto o venza el tiempo (s reales)."""
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def lookup(self, parent, child):
        try:
            return self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None


def report(ok, text):
    print(f"{'OK   ' if ok else 'FALLO'} {text}")
    return ok


def check_cloud(node):
    """Los puntos de la nube, llevados a base_footprint, deben quedar delante del robot."""
    msg = node.cloud_msg
    pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
    tf = node.lookup('base_footprint', msg.header.frame_id)
    if tf is None or len(pts) == 0:
        return report(False, f'nube de profundidad: {len(pts)} puntos válidos, sin TF')
    rot = _quat_to_matrix(tf.transform.rotation)
    trans = tf.transform.translation
    base = pts @ rot.T + np.array([trans.x, trans.y, trans.z])
    ahead = float(np.mean(base[:, 0] > 0.15))
    ground_z = float(np.percentile(base[:, 2], 5))
    return report(ahead > 0.95 and abs(ground_z) < 0.1,
                  f'nube /camera/depth/points: {len(pts)} puntos, frame {msg.header.frame_id}, '
                  f'{ahead * 100:.0f}% delante del robot, suelo en z≈{ground_z:.2f} m')


def navigate(node, x, y, timeout):
    """Envía una meta NavigateToPose y espera el resultado."""
    if not node.nav_client.wait_for_server(timeout_sec=60.0):
        return report(False, 'el servidor de acción navigate_to_pose no está disponible')
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0
    if not node.spin_until(lambda: node.lookup('map', 'base_footprint') is not None, 60.0):
        return report(False, 'no hay TF map → base_footprint')
    start = node.lookup('map', 'base_footprint').transform.translation
    print(f'      Meta ({x:.2f}, {y:.2f}) desde ({start.x:.2f}, {start.y:.2f}), '
          f'distancia recta {math.hypot(x - start.x, y - start.y):.2f} m')
    send = node.nav_client.send_goal_async(goal)
    node.spin_until(send.done, 10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        return report(False, 'la meta fue rechazada')
    result = handle.get_result_async()
    t0 = time.monotonic()
    if not node.spin_until(result.done, timeout):
        handle.cancel_goal_async()
        return report(False, f'no llegó a la meta en {timeout:.0f} s')
    status = result.result().status
    end = node.lookup('map', 'base_footprint').transform.translation
    error = math.hypot(x - end.x, y - end.y)
    return report(status == GoalStatus.STATUS_SUCCEEDED and error < 0.5,
                  f'NavigateToPose: estado {status} (4 = éxito) en {time.monotonic() - t0:.0f} s,'
                  f' error final {error:.2f} m')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--goal', type=float, nargs=2, default=(6.2, 3.2))
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavSmokeTest()
    ok = True

    for parent, child in TF_CHAIN:
        found = node.spin_until(lambda p=parent, c=child: node.lookup(p, c) is not None, 120.0)
        ok &= report(found, f'TF {parent} → {child}')
    if node.spin_until(lambda: node.map_msg is not None, 60.0):
        info = node.map_msg.info
        ok &= report(info.resolution == np.float32(0.05),
                     f'/map: publicado, resolución {info.resolution:.2f} m')
    else:
        ok &= report(False, '/map no publica')
    if node.spin_until(lambda: node.cloud_msg is not None, 30.0):
        ok &= check_cloud(node)
    else:
        ok &= report(False, '/camera/depth/points no publica')

    ok &= navigate(node, args.goal[0], args.goal[1], args.timeout)
    node.spin_until(lambda: False, 3.0)  # deja llegar el /map más reciente
    if node.map_msg is not None:
        data = np.array(node.map_msg.data)
        cell = node.map_msg.info.resolution ** 2
        print(f'      /map tras navegar: {np.count_nonzero(data == 0) * cell:.1f} m² libres, '
              f'{np.count_nonzero(data == 100) * cell:.1f} m² ocupados')

    node.destroy_node()
    rclpy.shutdown()
    print('RESULTADO:', 'TODO OK' if ok else 'HAY FALLOS')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
