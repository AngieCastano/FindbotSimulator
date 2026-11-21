"""
Nodo odom_noise_sim: odometría de ruedas con el ruido de los encoders reales.

Entrada:  /sim/wheel_odom (nav_msgs/Odometry de DiffDrive: encoders ideales + patinaje)
Salida:   /odom (nav_msgs/Odometry, frame odom → base_footprint) y la TF odom → base_footprint

/odom tiene el mismo formato que publicará el ESP32 con los encoders reales, de modo que SLAM y
Nav2 no distinguen la simulación del robot. Solo lee la odometría de las ruedas, nunca la pose
real de Gazebo. Solo corre en simulación.
"""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from usar_sim_sensors.odom_model import OdomNoiseModel

PARAMS = {
    'input_topic': '/sim/wheel_odom',
    'output_topic': '/odom',
    'publish_tf': True,
    'k_ss': 0.0005,
    'k_st': 0.0001,
    'k_ts': 0.00005,
    'k_tt': 0.00005,
    'sigma_v': 0.005,
    'sigma_w': 0.01,
    'seed': 0,
}


def yaw_of(q):
    """Ángulo de guiñada de un cuaternión."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomNoiseSim(Node):
    """Convierte la odometría ideal de Gazebo en la de los encoders reales."""

    def __init__(self):
        super().__init__('odom_noise_sim')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        self.model = OdomNoiseModel(self.p['k_ss'], self.p['k_st'], self.p['k_ts'],
                                    self.p['k_tt'], self.p['sigma_v'], self.p['sigma_w'],
                                    int(self.p['seed']))
        self.pub = self.create_publisher(Odometry, self.p['output_topic'], 50)
        self.tf = TransformBroadcaster(self) if self.p['publish_tf'] else None
        self.create_subscription(Odometry, self.p['input_topic'], self._on_odom, 50)
        self.get_logger().info(
            f'{self.p["input_topic"]} → {self.p["output_topic"]} (k_ss {self.p["k_ss"]} m²/m, '
            f'k_tt {self.p["k_tt"]} rad²/rad, semilla {self.p["seed"]})')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        x, y, yaw, v, w = self.model.step(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                          yaw_of(q), msg.twist.twist.linear.x,
                                          msg.twist.twist.angular.z)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = w
        # Covarianzas: incertidumbre acumulada de la pose y ruido instantáneo de la velocidad.
        big = 1e6  # z, roll y pitch no se estiman (robot planar)
        out.pose.covariance = [0.0] * 36
        out.twist.covariance = [0.0] * 36
        for i, value in enumerate([self.model.var_xy, self.model.var_xy, big, big, big,
                                   self.model.var_yaw]):
            out.pose.covariance[7 * i] = value
        for i, value in enumerate([self.p['sigma_v'] ** 2, big, big, big, big,
                                   self.p['sigma_w'] ** 2]):
            out.twist.covariance[7 * i] = value
        self.pub.publish(out)
        if self.tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = msg.child_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomNoiseSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
