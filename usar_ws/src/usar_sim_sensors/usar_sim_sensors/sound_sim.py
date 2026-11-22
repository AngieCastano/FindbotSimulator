"""
Nodo sound_sim: dirección de llegada del sonido que estimaría el ReSpeaker (Capa 2, simulación).

Entradas (excepción del ground truth, ver CLAUDE.md):
  ground_truth.yaml                    víctimas (fuentes) y escombros (atenúan el sonido)
  /sim/ground_truth/odom               pose y velocidad reales del robot (ruido de motores)
  TF base_footprint → mic_array_link   posición del arreglo en el robot
Salida:
  /sound/doa (usar_msgs/SoundDirection, frame mic_array_link) a `rate` Hz

Publica dirección, nivel y detección de voz, como el procesador del ReSpeaker real.
"""

import math

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from usar_msgs.msg import SoundDirection
from usar_sim_sensors.sound_model import DEFAULTS, MicArrayModel, VictimSounds
import yaml

PARAMS = {
    'ground_truth_file': '',
    'pose_topic': '/sim/ground_truth/odom',
    'output_topic': '/sound/doa',
    'base_frame': 'base_footprint',
    'sensor_frame': 'mic_array_link',
    'rate': 10.0,
    'moving_speed': 0.02,
    'moving_turn_rate': 0.05,
    'seed': 0,
    **DEFAULTS,
}


class SoundSim(Node):
    """Publica la dirección de llegada del sonido en la posición real del arreglo."""

    def __init__(self):
        super().__init__('sound_sim')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        if not self.p['ground_truth_file']:
            raise ValueError('sound_sim necesita ground_truth_file')
        with open(self.p['ground_truth_file'], encoding='utf-8') as stream:
            gt = yaml.safe_load(stream)
        victims = [(float(v['x']), float(v['y'])) for v in gt.get('victims', [])]
        rng = np.random.default_rng(int(self.p['seed']))
        # Estado de las víctimas: parámetro del escenario (ground truth) si lo trae; si no, YAML.
        vocal = float(gt.get('params', {}).get('vocal_fraction', self.p['vocal_fraction']))
        self.sounds = VictimSounds(len(victims), vocal,
                                   self.p['burst_duration'], self.p['burst_interval'], rng)
        self.model = MicArrayModel(victims, gt.get('obstacles', []),
                                   {k: self.p[k] for k in DEFAULTS}, rng)
        self.odom = None
        self.offset = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.p['pose_topic'],
                                 lambda m: setattr(self, 'odom', m), 10)
        self.pub = self.create_publisher(SoundDirection, self.p['output_topic'], 10)
        self.create_timer(1.0 / float(self.p['rate']), self._tick)
        self.get_logger().info(
            f'Sonido simulado: {len(self.sounds.vocal)} de {len(victims)} víctimas hacen ruido '
            f'({self.p["source_db"]:.0f} dB a 1 m, ráfagas de {self.p["burst_duration"]} s)')

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
        pose, twist = self.odom.pose.pose, self.odom.twist.twist
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        mic = (pose.position.x + math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1],
               pose.position.y + math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1])
        moving = (math.hypot(twist.linear.x, twist.linear.y) > self.p['moving_speed']
                  or abs(twist.angular.z) > self.p['moving_turn_rate'])
        azimuth, level, voice, confidence = self.model.measure(
            mic, yaw, moving, self.sounds.active(sec))
        msg = SoundDirection()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.p['sensor_frame']
        msg.azimuth = float(azimuth)
        msg.level_db = float(level)
        msg.voice_activity = bool(voice)
        msg.confidence = float(confidence)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SoundSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
