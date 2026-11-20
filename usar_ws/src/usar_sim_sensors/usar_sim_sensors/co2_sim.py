"""
Nodo co2_sim: medida simulada del sensor de CO₂ Figaro TGS4161 (Capa 2, solo simulación).

Entradas (excepción del ground truth, ver CLAUDE.md):
  ground_truth.yaml               posición de las víctimas (fuentes de CO₂)
  /sim/ground_truth/odom          pose real del robot en el mundo (Gazebo)
  TF base_footprint → co2_link    posición del sensor en el robot (modelo del robot)
Salida:
  /co2/concentration (usar_msgs/GasConcentration, frame co2_link) a `rate` Hz

Publica solo la concentración (ppm), como el ESP32 con el sensor real: el detector no recibe
ninguna posición de víctima.
"""

import math

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from usar_msgs.msg import GasConcentration
from usar_sim_sensors.co2_model import Co2Plume, Tgs4161
import yaml

PARAMS = {
    'ground_truth_file': '',
    'pose_topic': '/sim/ground_truth/odom',
    'output_topic': '/co2/concentration',
    'base_frame': 'base_footprint',
    'sensor_frame': 'co2_link',
    'rate': 2.0,
    'ambient_ppm': 400.0,
    'peak_excess_ppm': 2000.0,
    'decay_length': 1.0,
    'turbulence_sigma': 0.3,
    'turbulence_tau': 5.0,
    'response_tau': 20.0,
    'noise_log': 0.02,
    'min_ppm': 350.0,
    'max_ppm': 10000.0,
    'seed': 0,
}


def load_sources(path):
    """Posiciones (x, y) de las víctimas del ground truth (fuentes de CO₂)."""
    with open(path, encoding='utf-8') as stream:
        gt = yaml.safe_load(stream)
    return [(float(v['x']), float(v['y'])) for v in gt.get('victims', [])]


class Co2Sim(Node):
    """Publica la lectura del TGS4161 en la posición real del sensor."""

    def __init__(self):
        super().__init__('co2_sim')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        if not self.p['ground_truth_file']:
            raise ValueError('co2_sim necesita ground_truth_file')
        rng = np.random.default_rng(int(self.p['seed']))
        self.plume = Co2Plume(load_sources(self.p['ground_truth_file']), self.p['ambient_ppm'],
                              self.p['peak_excess_ppm'], self.p['decay_length'],
                              self.p['turbulence_sigma'], self.p['turbulence_tau'], rng)
        self.sensor = Tgs4161(self.p['response_tau'], self.p['noise_log'], self.p['min_ppm'],
                              self.p['max_ppm'], rng, initial_ppm=self.p['ambient_ppm'])
        self.pose = None
        self.offset = None
        self.last = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.p['pose_topic'],
                                 lambda m: setattr(self, 'pose', m.pose.pose), 10)
        self.pub = self.create_publisher(GasConcentration, self.p['output_topic'], 10)
        self.create_timer(1.0 / float(self.p['rate']), self._tick)
        self.get_logger().info(
            f'CO₂ simulado: {len(self.plume.sources)} fuentes, ambiente '
            f'{self.p["ambient_ppm"]:.0f} ppm, τ del sensor {self.p["response_tau"]} s')

    def _sensor_offset(self):
        """(x, y) del sensor en el frame de la base (fijo: se consulta una vez)."""
        if self.offset is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.p['base_frame'],
                                                     self.p['sensor_frame'], Time())
            except tf2_ros.TransformException:
                return None
            self.offset = (tf.transform.translation.x, tf.transform.translation.y)
        return self.offset

    def _tick(self):
        now = self.get_clock().now()
        sec = now.nanoseconds * 1e-9
        offset = self._sensor_offset()
        if self.pose is None or offset is None or sec <= 0.0:
            return
        q = self.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = self.pose.position.x + math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1]
        y = self.pose.position.y + math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1]
        dt = 0.0 if self.last is None else sec - self.last
        self.last = sec
        reading = self.sensor.update(self.plume.sample(x, y, dt), dt)
        msg = GasConcentration()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.p['sensor_frame']
        msg.gas = 'CO2'
        msg.concentration = float(reading)
        msg.variance = float((self.p['noise_log'] * reading) ** 2)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Co2Sim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
