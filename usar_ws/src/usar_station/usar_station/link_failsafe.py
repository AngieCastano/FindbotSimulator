"""
Nodo link_failsafe (a bordo del robot): parada segura si se pierde el enlace con la estación.

La estación envía un latido (/station/heartbeat → enlace → /station_heartbeat, std_msgs/Header
con la hora de envío). Si deja de llegar durante `timeout` s:
  1. cancela todas las metas de Nav2 (servicio de cancelación de NavigateToPose con id vacío),
  2. manda velocidad cero por /cmd_vel_teleop (el vigilante del firmware, CMD_TIMEOUT_MS,
     detiene además los motores si dejan de llegar mandos).
Al volver el enlace, el robot espera nuevas órdenes. Publica su estado en /link_failsafe/state
y, si log_csv, registra cada latido (hora de envío y de llegada: latencia estación → robot).
"""

import csv

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header, String


class LinkFailsafe(Node):
    """Detiene el robot cuando deja de recibir el latido de la estación."""

    def __init__(self):
        super().__init__('link_failsafe')
        self.timeout = float(self.declare_parameter('timeout', 1.0).value)
        cancel = self.declare_parameter(
            'cancel_service', '/navigate_to_pose/_action/cancel_goal').value
        self.cancel = self.create_client(CancelGoal, cancel)
        self.stop_pub = self.create_publisher(
            Twist, self.declare_parameter('stop_topic', '/cmd_vel_teleop').value, 10)
        self.state_pub = self.create_publisher(String, '/link_failsafe/state', 10)
        self.create_subscription(Header, self.declare_parameter(
            'heartbeat_topic', '/station_heartbeat').value, self._on_heartbeat, 10)
        self.create_timer(0.1, self._check)
        self.last = None
        self.lost = False
        self.events = 0
        path = self.declare_parameter('log_csv', '').value
        self.log = None
        if path:
            self.log_file = open(path, 'w', newline='', encoding='utf-8')
            self.log = csv.writer(self.log_file)
            self.log.writerow(['sent', 'received'])

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_heartbeat(self, msg):
        now = self._now()
        self.last = now
        if self.log is not None:
            self.log.writerow([f'{msg.stamp.sec + msg.stamp.nanosec * 1e-9:.4f}', f'{now:.4f}'])
        if self.lost:
            self.lost = False
            self.get_logger().info('Enlace recuperado: el robot espera nuevas órdenes')
            self.state_pub.publish(String(data='ok'))

    def _check(self):
        if self.last is None or self.lost or self._now() - self.last <= self.timeout:
            return
        self.lost = True
        self.events += 1
        self.get_logger().warn(
            f'Enlace perdido ({self.timeout:.1f} s sin latido): se cancelan las metas y se '
            f'detiene el robot (evento {self.events})')
        if self.cancel.service_is_ready():
            self.cancel.call_async(CancelGoal.Request())     # id vacío: cancela todas
        self.stop_pub.publish(Twist())
        self.state_pub.publish(String(data='lost'))

    def destroy_node(self):
        """Cierra el registro antes de terminar."""
        if self.log is not None:
            self.log_file.close()
            self.log = None
        super().destroy_node()


def main():
    rclpy.init()
    node = LinkFailsafe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
