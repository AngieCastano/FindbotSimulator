#!/usr/bin/env python3
"""
Prueba de humo de la simulación (Fase 2).

Con sim.launch.py ya corriendo, verifica que:
  - los sensores y la odometría publican (frecuencia medida en tiempo de simulación),
  - existen las TF odom→base_footprint→sensores,
  - el robot avanza al publicar en /cmd_vel.

Uso (dentro del contenedor, con el workspace cargado):
    python3 scripts/smoke_test_sim.py
Sale con código 0 si todo está bien.
"""

import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
import tf2_ros

# tópico → (tipo, frecuencia mínima esperada en Hz de simulación)
TOPICS = {
    '/scan': (LaserScan, 8.0),
    '/odom': (Odometry, 20.0),
    '/joint_states': (JointState, 20.0),
    '/camera/image': (Image, 10.0),
    '/camera/depth_image': (Image, 10.0),
    '/camera/camera_info': (CameraInfo, 10.0),
}
TF_PAIRS = [('odom', 'base_footprint'), ('base_footprint', 'lidar_link'),
            ('base_footprint', 'camera_optical_frame'),
            ('base_link', 'left_wheel_link')]
FORWARD_SPEED = 0.3  # m/s
FORWARD_TIME = 3.0   # s de simulación


class SmokeTest(Node):

    def __init__(self):
        super().__init__('smoke_test_sim', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.counts = {name: 0 for name in TOPICS}
        self.last = {}
        for name, (msg_type, _) in TOPICS.items():
            self.create_subscription(msg_type, name, self._make_cb(name),
                                     qos_profile_sensor_data)
        self.sim_time = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _make_cb(self, name):
        def _cb(msg):
            self.counts[name] += 1
            self.last[name] = msg
        return _cb

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def spin_for(self, wall_seconds):
        end = time.monotonic() + wall_seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_until_sim(self, sim_seconds, timeout=60.0, callback=None):
        start = self.sim_time
        deadline = time.monotonic() + timeout
        while self.sim_time - start < sim_seconds and time.monotonic() < deadline:
            if callback:
                callback()
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.sim_time - start


def main():
    rclpy.init()
    node = SmokeTest()
    ok = True

    deadline = time.monotonic() + 90
    while node.sim_time is None or not all(node.counts.values()):
        if time.monotonic() > deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.sim_time is None:
        print('FALLO: no llega /clock (¿está corriendo sim.launch.py?)')
        return 1

    # Frecuencias medidas sobre 5 s de simulación.
    node.counts = {name: 0 for name in TOPICS}
    wall_start = time.monotonic()
    sim_elapsed = node.spin_until_sim(5.0)
    rtf = sim_elapsed / (time.monotonic() - wall_start)
    print(f'Factor de tiempo real: {rtf:.2f}')
    for name, (_, min_rate) in TOPICS.items():
        rate = node.counts[name] / sim_elapsed
        good = rate >= min_rate
        ok &= good
        print(f"{'OK   ' if good else 'FALLO'} {name:24s} {rate:6.1f} Hz (mín. {min_rate})")

    scan = node.last.get('/scan')
    if scan:
        finite = [r for r in scan.ranges if math.isfinite(r)]
        print(f'      /scan: {len(scan.ranges)} rayos, frame {scan.header.frame_id}, '
              f'rango [{scan.range_min}, {scan.range_max}], {len(finite)} con eco')
    image = node.last.get('/camera/image')
    if image:
        print(f'      /camera/image: {image.width}x{image.height} {image.encoding}, '
              f'frame {image.header.frame_id}')
    depth = node.last.get('/camera/depth_image')
    if depth:
        print(f'      /camera/depth_image: {depth.encoding}, frame {depth.header.frame_id}')

    for parent, child in TF_PAIRS:
        good = node.tf_buffer.can_transform(parent, child, rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=2.0))
        ok &= good
        print(f"{'OK   ' if good else 'FALLO'} TF {parent} → {child}")

    start = node.last['/odom'].pose.pose.position
    twist = Twist()
    twist.linear.x = FORWARD_SPEED
    node.spin_until_sim(FORWARD_TIME, callback=lambda: node.cmd_pub.publish(twist))
    node.cmd_pub.publish(Twist())
    node.spin_until_sim(1.0, callback=lambda: node.cmd_pub.publish(Twist()))
    end = node.last['/odom'].pose.pose.position
    moved = math.hypot(end.x - start.x, end.y - start.y)
    good = moved > 0.5 * FORWARD_SPEED * FORWARD_TIME
    ok &= good
    print(f"{'OK   ' if good else 'FALLO'} /cmd_vel: el robot avanzó {moved:.2f} m "
          f'(orden: {FORWARD_SPEED} m/s durante {FORWARD_TIME} s)')

    node.destroy_node()
    rclpy.shutdown()
    print('RESULTADO:', 'TODO OK' if ok else 'HAY FALLOS')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
