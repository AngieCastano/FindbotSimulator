"""
Nodo radar_sim: señal IF del radar Doppler HB100 tras el acondicionamiento (Capa 2, simulación).

Entradas (excepción del ground truth, ver CLAUDE.md):
  ground_truth.yaml                víctimas (respiran) y escombros (atenúan las microondas)
  /sim/ground_truth/odom           pose y velocidad reales del robot (ecos del escenario en marcha)
  TF base_footprint → radar_link   posición de la antena en el robot
Salida:
  /radar/signal (usar_msgs/RadarSignal, frame radar_link): bloques de `block_size` muestras a
  `sample_rate` Hz, como los publicará el ESP32.

Solo publica la tensión medida: nunca posiciones, distancias ni identificadores.
"""

import math

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from usar_msgs.msg import RadarSignal
from usar_sim_sensors.radar_model import BreathingVictims, DEFAULTS, HB100Model
import yaml

PARAMS = {
    'ground_truth_file': '',
    'pose_topic': '/sim/ground_truth/odom',
    'output_topic': '/radar/signal',
    'base_frame': 'base_footprint',
    'sensor_frame': 'radar_link',
    'block_size': 20,
    'clutter_radius': 0.5,
    'seed': 0,
    **DEFAULTS,
}


class RadarSim(Node):
    """Publica la señal del HB100 en la posición real de la antena."""

    def __init__(self):
        super().__init__('radar_sim')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        if not self.p['ground_truth_file']:
            raise ValueError('radar_sim necesita ground_truth_file')
        with open(self.p['ground_truth_file'], encoding='utf-8') as stream:
            gt = yaml.safe_load(stream)
        victims = [(float(v['x']), float(v['y'])) for v in gt.get('victims', [])]
        rng = np.random.default_rng(int(self.p['seed']))
        model = {k: self.p[k] for k in DEFAULTS}
        breathing = BreathingVictims(len(victims), model['breathing_fraction'],
                                     model['rate_bpm'], model['displacement'], rng)
        self.model = HB100Model(victims, gt.get('obstacles', []), breathing, model, rng)
        self.rate = float(model['adc_rate']) / int(model['decimation'])
        self.odom = None
        self.offset = None
        self.t_next = None
        self.last_speed = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.p['pose_topic'],
                                 lambda m: setattr(self, 'odom', m), 10)
        self.pub = self.create_publisher(RadarSignal, self.p['output_topic'], 10)
        self.create_timer(int(self.p['block_size']) / self.rate, self._tick)
        self.get_logger().info(
            f'Radar simulado: {len(breathing.breathing)} de {len(victims)} víctimas respiran; '
            f'{self.rate:.0f} Hz en bloques de {self.p["block_size"]} muestras')

    def _sensor_offset(self):
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
        if self.odom is None or offset is None or sec <= 0.0:
            return
        n = int(self.p['block_size'])
        if self.t_next is None or abs(sec - self.t_next) > 1.0:
            self.t_next = sec                       # inicio, o salto del reloj simulado
        t0 = self.t_next - n / self.rate
        self.t_next += n / self.rate
        pose, twist = self.odom.pose.pose, self.odom.twist.twist
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        antenna = (pose.position.x + math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1],
                   pose.position.y + math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1])
        speed = math.hypot(twist.linear.x, twist.linear.y) \
            + abs(twist.angular.z) * self.p['clutter_radius']
        samples, saturated = self.model.block(t0, n, antenna, yaw, speed, self.last_speed)
        self.last_speed = speed
        msg = RadarSignal()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.p['sensor_frame']
        msg.sample_rate = float(self.rate)
        msg.samples = [float(v) for v in samples]
        msg.saturated = bool(saturated)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RadarSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
