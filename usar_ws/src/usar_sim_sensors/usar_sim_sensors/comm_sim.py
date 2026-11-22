"""
Nodo comm_sim: potencia de la señal WiFi entre la estación remota y el robot (Capa 2).

Entradas (excepción del ground truth, ver CLAUDE.md):
  ground_truth.yaml          escombros (atenúan la señal) y límites del área (muro perimetral)
  /sim/ground_truth/odom     pose real del robot
Salida:
  /comm/link (usar_msgs/LinkQuality, RSSI en dBm) a `rate` Hz: lo que reportaría la radio WiFi.

Publica solo la potencia recibida. Lo que pasa con los mensajes (retrasos, pérdidas) lo decide
usar_station/comm_relay a partir de esta medida, sin conocer el ground truth.
"""

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from usar_msgs.msg import LinkQuality
from usar_sim_sensors.comm_model import DEFAULTS, WifiLinkModel
import yaml

PARAMS = {
    'ground_truth_file': '',
    'pose_topic': '/sim/ground_truth/odom',
    'output_topic': '/comm/link',
    'station_x': -12.0,
    'station_y': 0.0,
    'rate': 10.0,
    'seed': 0,
    **DEFAULTS,
}


class CommSim(Node):
    """Publica el RSSI del enlace en la posición real del robot."""

    def __init__(self):
        super().__init__('comm_sim')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        if not self.p['ground_truth_file']:
            raise ValueError('comm_sim necesita ground_truth_file')
        with open(self.p['ground_truth_file'], encoding='utf-8') as stream:
            gt = yaml.safe_load(stream)
        self.model = WifiLinkModel(gt.get('obstacles', []), gt.get('bounds'),
                                   {k: self.p[k] for k in DEFAULTS},
                                   np.random.default_rng(int(self.p['seed'])))
        self.station = (float(self.p['station_x']), float(self.p['station_y']))
        self.robot = None
        self.create_subscription(Odometry, self.p['pose_topic'], self._on_pose, 10)
        self.pub = self.create_publisher(LinkQuality, self.p['output_topic'], 10)
        self.create_timer(1.0 / float(self.p['rate']), self._tick)
        self.get_logger().info(
            f'Enlace WiFi simulado: estación en ({self.station[0]:.1f}, {self.station[1]:.1f}) m, '
            f'n = {self.p["path_loss_exponent"]}, {self.p["obstacle_attenuation_db"]} dB por '
            'escombro')

    def _on_pose(self, msg):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tick(self):
        if self.robot is None:
            return
        out = LinkQuality()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'wifi'
        out.rssi_dbm = float(self.model.measure(self.station, self.robot,
                                                1.0 / float(self.p['rate'])))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CommSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
