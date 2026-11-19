"""
Nodo frontier_explorer: exploración autónoma por fronteras con J(g) = α·C(g) − β·I(g).

Entradas (solo datos propios del robot):
  /map                     mapa de slam_toolbox
  /global_costmap/costmap  costmap global de Nav2 (holgura de las metas)
  TF map → base_footprint  pose estimada por SLAM
Acciones usadas: compute_path_to_pose (C(g)) y navigate_to_pose (ejecutar la meta).
Salidas: /exploration/status (std_msgs/String) y /exploration/frontiers (MarkerArray).

Regla de diseño: este nodo NUNCA lee ground_truth.yaml ni poses de Gazebo.
"""

import math
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
import tf2_ros
from usar_exploration import frontier_core as fc
from visualization_msgs.msg import Marker, MarkerArray

PARAMS = {
    'alpha': 1.0,
    'beta': 1.0,
    'map_topic': '/map',
    'costmap_topic': '/global_costmap/costmap',
    'global_frame': 'map',
    'robot_frame': 'base_footprint',
    'free_threshold': 20,
    'occupied_threshold': 65,
    'min_cluster_size': 8,
    'segment_size': 1.5,
    'info_radius': 1.0,
    'goal_search_radius': 1.0,
    'min_goal_clearance': 0.45,
    'max_goal_cost': 70,
    'max_candidates': 6,
    'min_goal_distance': 0.5,
    'blacklist_radius': 0.6,
    'plan_timeout': 3.0,
    'plan_fail_penalty': 3.0,
    'replan_period': 5.0,
    'min_info_gain': 10.0,
    'progress_distance': 0.3,
    'progress_timeout': 20.0,
    'goal_timeout': 120.0,
    'no_frontier_timeout': 10.0,
    'max_time': 900.0,
    'tick_period': 0.5,
    'rejection_retry_delay': 2.0,
    'quick_failure_time': 3.0,
    'escape_cmd_topic': '/cmd_vel_teleop',
    'escape_cost_threshold': 99,
    'escape_probe_distances': [0.15, 0.3, 0.45],
    'escape_distance': 0.25,
    'escape_speed': 0.12,
    'escape_turn_speed': 0.6,
    'escape_timeout': 15.0,
    'max_rejections': 5,
}

WAITING, IDLE, PLANNING, NAVIGATING, ESCAPING, FINISHED = (
    'waiting', 'idle', 'planning', 'navigating', 'escaping', 'finished')


class FrontierExplorer(Node):
    """Explorador por fronteras: elige la meta de menor J(g) y la envía a Nav2."""

    def __init__(self):
        super().__init__('frontier_explorer')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMS}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_msg: Optional[OccupancyGrid] = None
        self.costmap_msg: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, self.p['map_topic'],
                                 lambda m: setattr(self, 'map_msg', m), latched)
        self.create_subscription(OccupancyGrid, self.p['costmap_topic'],
                                 lambda m: setattr(self, 'costmap_msg', m),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.status_pub = self.create_publisher(String, '/exploration/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        # Maniobra de escape: pasa por el collision_monitor de órdenes externas a Nav2.
        self.escape_pub = self.create_publisher(Twist, self.p['escape_cmd_topic'], 10)
        self.escape = None
        self.escapes = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.state = WAITING
        self.status = 'waiting'
        self.start_time: Optional[Time] = None
        self.no_frontier_since: Optional[Time] = None
        self.blacklist: List[tuple] = []
        self.candidates: List[fc.Frontier] = []
        self.frontiers: List[fc.Frontier] = []
        self.plan_index = 0
        self.plan_deadline: Optional[Time] = None
        self.goal: Optional[fc.Frontier] = None
        self.goal_handle = None
        self.goal_seq = 0
        self.goal_start: Optional[Time] = None
        self.last_replan: Optional[Time] = None
        self.progress_pose = None
        self.progress_time: Optional[Time] = None
        self.goals_sent = 0
        self.goals_failed = 0
        self.rejections = 0
        self.retry_after: Optional[Time] = None

        self.create_timer(self.p['tick_period'], self._tick)
        self.create_timer(1.0, self._publish_status)

    # ------------------------------------------------------------------ utilidades
    def _now(self) -> Time:
        return self.get_clock().now()

    def _elapsed(self, since: Optional[Time]) -> float:
        return (self._now() - since).nanoseconds * 1e-9 if since is not None else 0.0

    def _robot_pose(self):
        """Pose (x, y, yaw) del robot en el frame global según SLAM, o None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.p['global_frame'], self.p['robot_frame'], Time(),
                timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _robot_xy(self):
        pose = self._robot_pose()
        return pose[:2] if pose is not None else None

    @staticmethod
    def _grid(msg: OccupancyGrid):
        i = msg.info
        info = fc.GridInfo(i.resolution, i.origin.position.x, i.origin.position.y,
                           i.width, i.height)
        return np.asarray(msg.data, dtype=np.int16).reshape(i.height, i.width), info

    def _pose(self, x, y) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.p['global_frame']
        pose.header.stamp = self._now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------ ciclo principal
    def _tick(self):
        if self.state == FINISHED:
            return
        robot = self._robot_xy()
        if self.state == WAITING:
            if (self.map_msg is not None and robot is not None
                    and self.nav_client.server_is_ready() and self.plan_client.server_is_ready()):
                self.start_time = self._now()
                self.state = IDLE
                self.get_logger().info('Exploración iniciada')
            return
        if self._elapsed(self.start_time) > self.p['max_time']:
            self._finish('timeout')
            return
        if robot is None:
            return
        if self.state == IDLE:
            if self.retry_after is None or self._now() >= self.retry_after:
                if not self._start_escape_if_trapped():
                    self._select_candidates(robot)
        elif self.state == ESCAPING:
            self._escape_step()
        elif self.state == PLANNING:
            self._check_plan_timeout()
        elif self.state == NAVIGATING:
            self._monitor_goal(robot)

    # ------------------------------------------------------------------ selección de metas
    def _select_candidates(self, robot):
        """Detecta fronteras, calcula metas seguras e I(g) y pide C(g) para los K mejores."""
        grid, info = self._grid(self.map_msg)
        free, occupied, unknown = fc.classify(
            grid, self.p['free_threshold'], self.p['occupied_threshold'])
        frontiers = fc.cluster_frontiers(fc.frontier_mask(free, unknown), info,
                                         self.p['min_cluster_size'], self.p['segment_size'])
        gain = fc.information_map(unknown, round(self.p['info_radius'] / info.resolution))
        clearance = fc.clearance_map(occupied, info.resolution)
        cost_ok = None
        if self.costmap_msg is not None:
            cost, cost_info = self._grid(self.costmap_msg)
            cost_ok = fc.align_to(info, cost, cost_info, fill=-1) <= self.p['max_goal_cost']

        valid = []
        rejected = {'sin_holgura': 0, 'cerca': 0, 'poca_info': 0, 'lista_negra': 0}
        for f in frontiers:
            cell = fc.find_safe_goal(fc.representative_cell(f, info), free, clearance, info,
                                     self.p['goal_search_radius'],
                                     self.p['min_goal_clearance'], cost_ok)
            if cell is None:
                rejected['sin_holgura'] += 1
                continue
            f.goal = info.cell_to_world(*cell)
            f.info_gain = float(gain[cell])
            dist = math.hypot(f.goal[0] - robot[0], f.goal[1] - robot[1])
            reason = ('cerca' if dist < self.p['min_goal_distance'] else
                      'poca_info' if f.info_gain < self.p['min_info_gain'] else
                      'lista_negra' if fc.is_blacklisted(f.goal, self.blacklist,
                                                         self.p['blacklist_radius']) else None)
            if reason:
                rejected[reason] += 1
                continue
            f.extra['euclidean'] = dist
            valid.append(f)
        self.frontiers = frontiers

        if not valid:
            if self.no_frontier_since is None:
                self.get_logger().info(
                    f'Sin fronteras válidas ({len(frontiers)} fronteras; descartadas: '
                    + ', '.join(f'{k}={v}' for k, v in rejected.items()) + ')')
            self.no_frontier_since = self.no_frontier_since or self._now()
            if self._elapsed(self.no_frontier_since) > self.p['no_frontier_timeout']:
                self._finish('no_frontiers')
            self._publish_markers()
            return
        self.no_frontier_since = None
        valid.sort(key=lambda f: f.extra['euclidean'])
        self.candidates = valid[:self.p['max_candidates']]
        self.plan_index = 0
        self.state = PLANNING
        self._request_next_plan()

    def _request_next_plan(self):
        """Pide al planificador la ruta del siguiente candidato (una petición cada vez)."""
        if self.plan_index >= len(self.candidates):
            self._choose_and_send()
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(*self.candidates[self.plan_index].goal)
        goal.use_start = False
        self.plan_deadline = self._now() + Duration(seconds=self.p['plan_timeout'])
        index = self.plan_index
        self.plan_client.send_goal_async(goal).add_done_callback(
            lambda fut: self._on_plan_accepted(fut, index))

    def _on_plan_accepted(self, future, index):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._store_plan(index, None)
            return
        handle.get_result_async().add_done_callback(lambda fut: self._on_plan_result(fut, index))

    def _on_plan_result(self, future, index):
        result = future.result()
        path = None
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            path = [(p.pose.position.x, p.pose.position.y) for p in result.result.path.poses]
        self._store_plan(index, path)

    def _store_plan(self, index, path):
        """Guarda C(g): longitud de la ruta, o distancia euclidiana penalizada si no hay ruta."""
        if self.state != PLANNING or index != self.plan_index:
            return
        cand = self.candidates[index]
        if path:
            cand.path_cost = fc.path_length(path)
        else:
            cand.path_cost = cand.extra['euclidean'] * self.p['plan_fail_penalty']
            cand.extra['plan_failed'] = True
        self.plan_index += 1
        self._request_next_plan()

    def _check_plan_timeout(self):
        if self.plan_deadline is not None and self._now() > self.plan_deadline:
            self.get_logger().warn('El planificador no respondió a tiempo; se usa la distancia')
            self._store_plan(self.plan_index, None)

    def _choose_and_send(self):
        """Elige argmin J(g) y envía la meta a NavigateToPose."""
        fc.score_frontiers(self.candidates, self.p['alpha'], self.p['beta'])
        best = fc.best_frontier(self.candidates)
        if best is None:
            self.state = IDLE
            return
        self.goal = best
        self.goal_seq += 1
        seq = self.goal_seq
        self.goals_sent += 1
        self.get_logger().info(
            f'Meta {self.goals_sent}: ({best.goal[0]:.2f}, {best.goal[1]:.2f}) '
            f'J={best.score:.3f} C={best.path_cost:.2f} m I={best.info_gain:.0f} celdas '
            f'({len(self.candidates)} candidatos, {len(self.frontiers)} fronteras)')
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(*best.goal)
        self.state = NAVIGATING
        self.goal_start = self.last_replan = self.progress_time = self._now()
        self.progress_pose = self._robot_xy()
        self.goal_handle = None
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda fut: self._on_nav_accepted(fut, seq))
        self._publish_markers()

    # ------------------------------------------------------------------ ejecución de la meta
    def _on_nav_accepted(self, future, seq):
        if seq != self.goal_seq:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._on_rejected()
            return
        self.rejections = 0
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut: self._on_nav_result(fut, seq))

    def _on_nav_result(self, future, seq):
        if seq != self.goal_seq or self.state != NAVIGATING:
            return
        status = future.result().status if future.result() is not None else None
        success = status == GoalStatus.STATUS_SUCCEEDED
        if not success and self._elapsed(self.goal_start) < self.p['quick_failure_time']:
            # Fallo inmediato (p. ej. "Start occupied"): el problema es la situación del robot,
            # no la meta. Se reintenta (tras comprobar si hace falta escapar) sin vetarla.
            self._on_rejected()
            return
        self._goal_done(success, f'estado {status}')

    def _on_rejected(self):
        """
        Meta rechazada por Nav2 (p. ej. aún activándose): se reintenta sin penalizarla.

        Solo tras max_rejections rechazos seguidos se trata como meta fallida.
        """
        self.rejections += 1
        if self.rejections >= self.p['max_rejections']:
            self.rejections = 0
            self._goal_done(success=False, reason='rechazada por Nav2')
            return
        self.get_logger().warn(f'Nav2 rechazó o abortó al instante la meta ({self.rejections}); '
                               'se reintenta')
        self.goal, self.goal_handle = None, None
        self.goal_seq += 1
        self.goals_sent -= 1
        self.retry_after = self._now() + Duration(seconds=self.p['rejection_retry_delay'])
        self.state = IDLE

    def _monitor_goal(self, robot):
        """Vigila progreso, tiempo máximo por meta y si la frontera ya quedó explorada."""
        moved = math.hypot(robot[0] - self.progress_pose[0], robot[1] - self.progress_pose[1]) \
            if self.progress_pose else 0.0
        if moved >= self.p['progress_distance']:
            self.progress_pose, self.progress_time = robot, self._now()
        elif self._elapsed(self.progress_time) > self.p['progress_timeout']:
            self._cancel_goal(success=False, reason='sin progreso')
            return
        if self._elapsed(self.goal_start) > self.p['goal_timeout']:
            self._cancel_goal(success=False, reason='tiempo por meta agotado')
            return
        if self._elapsed(self.last_replan) >= self.p['replan_period']:
            self.last_replan = self._now()
            grid, info = self._grid(self.map_msg)
            cell = info.world_to_cell(*self.goal.goal)
            if cell is not None:
                _, _, unknown = fc.classify(grid, self.p['free_threshold'],
                                            self.p['occupied_threshold'])
                r = round(self.p['info_radius'] / info.resolution)
                kernel = fc.disk_kernel(r)
                window = np.pad(unknown, r)[cell[0]:cell[0] + 2 * r + 1,
                                            cell[1]:cell[1] + 2 * r + 1]
                if float((window & kernel).sum()) < self.p['min_info_gain']:
                    self._cancel_goal(success=True, reason='frontera ya explorada')

    def _cancel_goal(self, success, reason):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._goal_done(success, reason)

    def _goal_done(self, success, reason):
        """Cierra la meta actual; toda meta terminada pasa a la lista negra (no se repite)."""
        if self.goal is None:
            return
        if not success:
            self.goals_failed += 1
        self.blacklist.append(self.goal.goal)
        self.get_logger().info(
            f'Meta {"completada" if success else "fallida"} ({reason}); '
            f'lista negra: {len(self.blacklist)}')
        self.goal, self.goal_handle = None, None
        self.goal_seq += 1          # invalida callbacks pendientes de la meta anterior
        self.state = IDLE

    # ------------------------------------------------------------------ escape
    def _start_escape_if_trapped(self) -> bool:
        """
        Si el centro del robot está en la zona inscrita del costmap global, inicia el escape.

        Desde esa zona el planificador de Nav2 rechaza cualquier ruta ("Start occupied"), aunque
        físicamente haya espacio. Devuelve True si se inició la maniobra.
        """
        pose = self._robot_pose()
        if self.costmap_msg is None or pose is None:
            return False
        cost, info = self._grid(self.costmap_msg)
        if fc.cost_at(cost, info, pose[0], pose[1], outside=0) < self.p['escape_cost_threshold']:
            return False
        heading, forward = fc.escape_direction(cost, info, *pose,
                                               self.p['escape_probe_distances'])
        self.escapes += 1
        self.escape = {'heading': heading if forward else heading + math.pi,
                       'forward': forward, 'start': pose[:2], 'since': self._now(),
                       'turning': True}
        self.get_logger().info(
            f'Robot en zona inscrita del costmap: escape {self.escapes} hacia '
            f'{math.degrees(heading):.0f}° ({"avanzando" if forward else "retrocediendo"})')
        self.state = ESCAPING
        return True

    def _escape_step(self):
        """Gira hacia el rumbo de escape y se desplaza escape_distance (o hasta salir)."""
        pose = self._robot_pose()
        esc = self.escape
        cmd = Twist()
        done = pose is None or self._elapsed(esc['since']) > self.p['escape_timeout']
        if not done:
            delta = esc['heading'] - pose[2]
            err = math.atan2(math.sin(delta), math.cos(delta))
            if esc['turning'] and abs(err) > 0.15:
                cmd.angular.z = math.copysign(self.p['escape_turn_speed'], err)
            else:
                esc['turning'] = False
                moved = math.hypot(pose[0] - esc['start'][0], pose[1] - esc['start'][1])
                cost, info = self._grid(self.costmap_msg)
                free = (fc.cost_at(cost, info, pose[0], pose[1], 0)
                        < self.p['escape_cost_threshold'])
                if moved >= self.p['escape_distance'] or (free and moved > 0.05):
                    done = True
                else:
                    speed = self.p['escape_speed']
                    cmd.linear.x = speed if esc['forward'] else -speed
                    cmd.angular.z = 1.5 * err
        self.escape_pub.publish(Twist() if done else cmd)
        if done:
            self.escape = None
            self.state = IDLE

    # ------------------------------------------------------------------ fin y salidas
    def _finish(self, reason):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal, self.goal_handle = None, None
        self.goal_seq += 1
        self.state = FINISHED
        self.status = f'finished:{reason}'
        self.get_logger().info(
            f'Exploración terminada ({reason}) en {self._elapsed(self.start_time):.0f} s: '
            f'{self.goals_sent} metas enviadas, {self.goals_failed} fallidas')
        self._publish_status()
        self._publish_markers()

    def _publish_status(self):
        if self.state != FINISHED:
            self.status = 'exploring' if self.state != WAITING else 'waiting'
        self.status_pub.publish(String(data=self.status))

    def _publish_markers(self):
        frame = self.p['global_frame']
        stamp = self._now().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]

        def marker(mid, mtype, scale, color, ns):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            m.pose.orientation.w = 1.0
            return m

        if self.map_msg is not None and self.frontiers:
            _, info = self._grid(self.map_msg)
            cells = marker(0, Marker.POINTS, info.resolution, (0.0, 0.8, 1.0), 'frontier_cells')
            for f in self.frontiers:
                for row, col in f.cells:
                    x, y = info.cell_to_world(row, col)
                    cells.points.append(Point(x=x, y=y, z=0.02))
            markers.append(cells)
        for i, f in enumerate(self.candidates):
            if f.goal is not None:
                m = marker(i, Marker.SPHERE, 0.15, (1.0, 0.9, 0.0), 'candidates')
                m.pose.position.x, m.pose.position.y = f.goal
                markers.append(m)
        if self.goal is not None:
            m = marker(0, Marker.SPHERE, 0.3, (0.0, 1.0, 0.0), 'goal')
            m.pose.position.x, m.pose.position.y = self.goal.goal
            markers.append(m)
        for i, (x, y) in enumerate(self.blacklist):
            m = marker(i, Marker.SPHERE, 0.12, (1.0, 0.0, 0.0), 'blacklist')
            m.pose.position.x, m.pose.position.y = x, y
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
