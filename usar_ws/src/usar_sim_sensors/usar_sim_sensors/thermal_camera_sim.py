"""
Nodo thermal_camera_sim: imagen de la cámara térmica simulada con el ruido del MLX90640.

Entrada:  /sim/thermal/image_raw (sensor_msgs/Image mono16 de Gazebo, `resolution` K por unidad)
Salida:   /thermal/image (sensor_msgs/Image 32FC1, °C por píxel, frame thermal_optical_frame)

/thermal/image tiene el mismo formato que publicará el ESP32 con el MLX90640 real, de modo que
el detector no distingue la simulación del robot. Solo corre en simulación.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from usar_sim_sensors.thermal_model import raw_to_celsius, ThermalNoiseModel

PARAMS = {
    'input_topic': '/sim/thermal/image_raw',
    'output_topic': '/thermal/image',
    'resolution': 0.01,
    'width': 32,
    'height': 24,
    'netd': 0.2,
    'fixed_pattern': 0.3,
    'min_temp': -40.0,
    'max_temp': 300.0,
    'seed': 0,
}


class ThermalCameraSim(Node):
    """Convierte la imagen ideal de Gazebo en la del sensor real."""

    def __init__(self):
        super().__init__('thermal_camera_sim')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        self.model = ThermalNoiseModel(self.p['width'], self.p['height'], self.p['netd'],
                                       self.p['fixed_pattern'], self.p['min_temp'],
                                       self.p['max_temp'], self.p['seed'])
        self.pub = self.create_publisher(Image, self.p['output_topic'], qos_profile_sensor_data)
        self.create_subscription(Image, self.p['input_topic'], self._on_image,
                                 qos_profile_sensor_data)
        self.warned = False
        self.get_logger().info(
            f'{self.p["input_topic"]} → {self.p["output_topic"]} (°C, NETD '
            f'{self.p["netd"]} K, patrón fijo {self.p["fixed_pattern"]} K)')

    def _on_image(self, msg):
        expected = (self.p['width'], self.p['height'])
        if msg.encoding != 'mono16' or (msg.width, msg.height) != expected:
            if not self.warned:
                self.get_logger().error(
                    f'Imagen inesperada: {msg.encoding} {msg.width}×{msg.height}; se esperaba '
                    f'mono16 {self.p["width"]}×{self.p["height"]}')
                self.warned = True
            return
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        raw = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width)
        celsius = self.model.apply(raw_to_celsius(raw, self.p['resolution']))
        out = Image()
        out.header = msg.header
        out.height, out.width = celsius.shape
        out.encoding = '32FC1'
        out.is_bigendian = 0
        out.step = 4 * out.width
        out.data = celsius.astype('<f4').tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ThermalCameraSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
