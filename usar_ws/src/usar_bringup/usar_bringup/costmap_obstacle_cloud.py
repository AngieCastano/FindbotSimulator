"""
Publica las celdas letales del costmap local como nube de puntos.

El collision_monitor solo usa datos instantáneos de los sensores. Muy cerca del frontal hay un
punto ciego: el LiDAR (plano a 0.18 m) pasa por encima de las extremidades de las víctimas y la
cámara no ve a menos de su distancia mínima. El costmap local sí recuerda esas extremidades (la
capa de profundidad las conserva), así que esta nube le da "memoria" al filtro de colisión.

Solo usa datos propios del robot (/local_costmap/costmap); nunca el ground truth.
"""

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def lethal_cells_to_points(data, width, height, resolution, origin_xy, lethal_value, z):
    """Devuelve un arreglo N×3 con el centro de las celdas cuyo valor es >= lethal_value."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    rows, cols = np.nonzero(grid >= lethal_value)
    xs = origin_xy[0] + (cols + 0.5) * resolution
    ys = origin_xy[1] + (rows + 0.5) * resolution
    return np.column_stack([xs, ys, np.full(xs.shape, z)]).astype(np.float32)


class CostmapObstacleCloud(Node):
    """Convierte /local_costmap/costmap (OccupancyGrid) en una PointCloud2 de obstáculos."""

    def __init__(self):
        super().__init__('costmap_obstacle_cloud')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cloud_topic', '/local_costmap/obstacle_points')
        self.declare_parameter('lethal_value', 100)
        self.declare_parameter('point_height', 0.1)
        self.lethal = self.get_parameter('lethal_value').value
        self.z = float(self.get_parameter('point_height').value)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('cloud_topic').value, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value, self._on_costmap, qos)

    def _on_costmap(self, msg):
        info = msg.info
        points = lethal_cells_to_points(
            msg.data, info.width, info.height, info.resolution,
            (info.origin.position.x, info.origin.position.y), self.lethal, self.z)
        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))


def main():
    rclpy.init()
    node = CostmapObstacleCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
