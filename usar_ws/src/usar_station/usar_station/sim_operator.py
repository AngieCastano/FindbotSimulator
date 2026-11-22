"""
Nodo sim_operator (en la estación remota): el operador de la teleoperación.

En los dos modos:
  - envía el latido de la estación (/station/heartbeat, std_msgs/Header) para el failsafe,
  - publica el estado de la misión (/exploration/status: exploring → finished:<motivo>), que
    usa el evaluador para medir tiempos, y la termina a los `max_time` s,
  - registra (log_csv) la antigüedad con que llegan a la estación los datos del robot (odometría
    e imagen): latencia robot → estación.

Modo `human`: el operador real conduce con el teclado (scripts/station_teleop.sh, publica en
/station/cmd_vel) o marca metas con "2D Goal Pose" en el RViz de la estación.
Modo `simulated`: un operador simulado marca metas de barrido en zigzag por el área (carriles
cada `lane_spacing` m, el alcance aproximado de la cámara), como un rescatista que revisa la zona
punto por punto (reenvía la meta cada `goal_resend` s mientras el robot no arranque). Solo usa
lo que ve la estación (mapa y pose recibidos por el enlace) y el tamaño
del área (dato del sitio, no del ground truth). Pasa a la siguiente meta al llegar, si el robot
no avanza en `progress_timeout` s o si la meta supera `goal_timeout` s.
"""

import csv
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
import tf2_ros
from usar_station.sweep import free_near, sweep_waypoints

PARAMS = {
    'mode': 'simulated',
    'max_time': 900.0,
    'heartbeat_rate': 5.0,
    'area_size': [20.0, 20.0],
    'margin': 1.5,
    'lane_spacing': 3.0,
    'goal_tolerance': 1.0,
    'progress_distance': 0.3,
    'progress_timeout': 20.0,
    'goal_timeout': 90.0,
    'goal_resend': 5.0,
    'free_search_radius': 1.5,
    'global_frame': 'map',
    'robot_frame': 'base_footprint',
    'log_csv': '',
}


class SimOperator(Node):
    """Latido, estado de la misión y, en modo simulado, metas de barrido."""

    def __init__(self):
        super().__init__('sim_operator')
        self.p = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/exploration/status', latched)
        self.heartbeat_pub = self.create_publisher(Header, '/station/heartbeat', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/station/goal_pose', 10)
        # TF de la estación: la que llega por el enlace (/station/tf).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/station/map', self._on_map, latched)
        self.map = None
        self.log = None
        if self.p['log_csv']:
            self.log_file = open(self.p['log_csv'], 'w', newline='', encoding='utf-8')
            self.log = csv.writer(self.log_file)
            self.log.writerow(['topic', 'stamp', 'received'])
            self.create_subscription(Odometry, '/station/odometry/filtered',
                                     lambda m: self._log_age('odometry', m.header), 10)
            self.create_subscription(
                Image, '/station/camera/image', lambda m: self._log_age('camera', m.header),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start = None
        self.finished = False
        self.waypoints = []
        self.goal = None
        self.goals_sent = 0
        self.create_timer(1.0 / float(self.p['heartbeat_rate']), self._heartbeat)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Estación remota: operador {self.p["mode"]}, misión de '
                               f'{self.p["max_time"]:.0f} s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, msg):
        self.map = msg

    def _log_age(self, name, header):
        self.log.writerow([name, f'{header.stamp.sec + header.stamp.nanosec * 1e-9:.4f}',
                           f'{self._now():.4f}'])

    def _heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'station'
        self.heartbeat_pub.publish(msg)

    def _robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.p['global_frame'], self.p['robot_frame'],
                                                 Time())
        except tf2_ros.TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _tick(self):
        now = self._now()
        if now <= 0.0 or self.finished:
            return
        robot = self._robot()
        if self.start is None:
            if robot is None:
                return                       # aún no llegan datos del robot a la estación
            self.start = now
            self.status_pub.publish(String(data='exploring'))
            if self.p['mode'] == 'simulated':
                self.waypoints = sweep_waypoints(tuple(self.p['area_size']), self.p['margin'],
                                                 self.p['lane_spacing'], robot)
                self.get_logger().info(f'Barrido de {len(self.waypoints)} puntos')
        if now - self.start >= self.p['max_time']:
            self._finish('timeout')
            return
        if self.p['mode'] == 'simulated' and robot is not None:
            self._drive(now, robot)

    def _drive(self, now, robot):
        g = self.goal
        if g is not None:
            dist = math.hypot(robot[0] - g['xy'][0], robot[1] - g['xy'][1])
            if math.hypot(robot[0] - g['progress'][0], robot[1] - g['progress'][1]) \
                    >= self.p['progress_distance']:
                g['progress'], g['progress_t'] = robot, now
            reason = None
            if dist <= self.p['goal_tolerance']:
                reason = 'alcanzada'
            elif now - g['progress_t'] > self.p['progress_timeout']:
                reason = 'sin progreso'
            elif now - g['sent'] > self.p['goal_timeout']:
                reason = 'tiempo agotado'
            if reason is None:
                # Una meta puede perderse (Nav2 aún arrancando, enlace caído): se reenvía hasta
                # que el robot empiece a avanzar hacia ella.
                if g['progress_t'] == g['sent'] and now - g['last_send'] >= self.p['goal_resend']:
                    self.goal_pub.publish(g['msg'])
                    g['last_send'] = now
                return
            self.get_logger().info(f'Meta {self.goals_sent} {reason}')
            self.goal = None
        while self.waypoints:
            target = self.waypoints.pop(0)
            if self.map is not None:
                info = self.map.info
                grid = np.asarray(self.map.data, dtype=np.int16).reshape(info.height, info.width)
                target = free_near(grid, (info.origin.position.x, info.origin.position.y),
                                   info.resolution, target, self.p['free_search_radius'])
            if target is not None:
                break
        else:
            self._finish('route_done')
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['global_frame']
        msg.pose.position.x, msg.pose.position.y = float(target[0]), float(target[1])
        yaw = math.atan2(target[1] - robot[1], target[0] - robot[0])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(msg)
        self.goals_sent += 1
        self.goal = {'xy': target, 'sent': now, 'progress': robot, 'progress_t': now,
                     'msg': msg, 'last_send': now}
        self.get_logger().info(f'Meta {self.goals_sent}: ({target[0]:.2f}, {target[1]:.2f}); '
                               f'quedan {len(self.waypoints)}')

    def _finish(self, reason):
        self.finished = True
        self.status_pub.publish(String(data=f'finished:{reason}'))
        self.get_logger().info(f'Misión teleoperada terminada ({reason}) en '
                               f'{self._now() - (self.start or self._now()):.0f} s: '
                               f'{self.goals_sent} metas')

    def destroy_node(self):
        """Cierra el registro antes de terminar."""
        if self.log is not None:
            self.log_file.close()
            self.log = None
        super().destroy_node()


def main():
    rclpy.init()
    node = SimOperator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
