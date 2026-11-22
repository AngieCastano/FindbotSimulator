"""
Nodo frontier_explorer: exploración autónoma por fronteras con J(g) = α·C(g) − β·I(g).

Entradas (solo datos propios del robot):
  /map                     mapa de slam_toolbox
  /global_costmap/costmap  costmap global de Nav2 (holgura de las metas)
  TF map → base_footprint  pose estimada por SLAM
  /victims/candidates      candidatos a víctima de los detectores guía (p. ej. sonido): antes de
                           elegir otra frontera, el robot va a mirar cada uno (una vez)
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
from usar_msgs.msg import VictimDetectionArray
from visualization_msgs.msg import Marker, MarkerArray

PARAMS = {
    'alpha': 1.0,
    'beta': 1.0,
    'gamma': 0.0,
    'sign_sigma': 2.0,
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
    'min_goal_clearance': 0.38,
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
    'stuck_failures': 2,
    'escape_min_motion': 0.03,
    'escape_bypass_after': 3,
    'escape_bypass_topic': '/cmd_vel',
    'escape_bypass_speed': 0.05,
    'escape_bypass_distance': 0.15,
    'trap_timeout': 60.0,
    'trap_radius': 1.0,
    'backtrack_distance': 2.0,
    'backtrack_speed': 0.1,
    'backtrack_turn_speed': 0.5,
    'backtrack_lookahead': 0.3,
    'backtrack_timeout': 45.0,
    'backtrack_stall_time': 8.0,
    'trap_avoid_radius': 1.2,
    'crumb_spacing': 0.1,
    'control_period': 0.1,
    'listen_enabled': False,
    'listen_period': 45.0,
    'listen_duration': 5.0,
    'max_rejections': 5,
    'candidate_topic': '/victims/candidates',
    'candidate_visits_enabled': True,
    'candidate_view_min': 0.8,
    'candidate_view_max': 2.0,
    'candidate_done_radius': 1.0,
    'candidate_look_time': 4.0,
    'candidate_max_visits': 10,
}

(WAITING, IDLE, PLANNING, NAVIGATING, ESCAPING, BACKTRACKING, LISTENING, LOOKING,
 FINISHED) = ('waiting', 'idle', 'planning', 'navigating', 'escaping', 'backtracking',
              'listening', 'looking', 'finished')


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
        # Escape de último recurso (ver _start_escape_if_trapped): directo a la base del robot.
        self.bypass_pub = self.create_publisher(Twist, self.p['escape_bypass_topic'], 10)
        self.escape_failures = 0      # escapes seguidos que no movieron el robot
        self.escape = None
        self.escapes = 0
        # Metas seguidas fallidas sin progreso con el robot en el mismo sitio.
        self.stuck_count = 0
        self.stuck_anchor = None
        # Encierro: el robot no se aleja trap_radius de trap_anchor en trap_timeout segundos.
        self.crumbs = fc.Breadcrumbs(self.p['crumb_spacing'])
        self.trap_anchor = None
        self.trap_since: Optional[Time] = None
        self.trap_zones: List[tuple] = []
        self.backtrack = None
        self.backtracks = 0
        # Paradas de escucha (sonido, radar): el robot se detiene listen_duration s cada
        # listen_period s de navegación hacia metas.
        self.listen_last: Optional[Time] = None
        self.listen_start: Optional[Time] = None
        self.listens = 0
        # Candidatos a víctima (detectores guía): se visitan una vez, mirando hacia ellos.
        self.victim_candidates: List[tuple] = []
        self.visited_candidates: List[tuple] = []
        self.candidate_visits = 0
        self.look_start: Optional[Time] = None
        if self.p['candidate_visits_enabled']:
            self.create_subscription(VictimDetectionArray, self.p['candidate_topic'],
                                     self._on_candidates, 10)

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
        self.create_timer(self.p['control_period'], self._control)
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

    def _pose(self, x, y, yaw=None) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.p['global_frame']
        pose.header.stamp = self._now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        if yaw is None:
            pose.pose.orientation.w = 1.0
        else:
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ------------------------------------------------------------------ ciclo principal
    def _tick(self):
        if self.state == FINISHED:
            return
        robot = self._robot_xy()
        if self.state == WAITING:
            if (self.map_msg is not None and robot is not None
                    and self.nav_client.server_is_ready() and self.plan_client.server_is_ready()):
                self.start_time = self.listen_last = self._now()
                self.state = IDLE
                self.get_logger().info('Exploración iniciada')
            return
        if self._elapsed(self.start_time) > self.p['max_time']:
            self._finish('timeout')
            return
        if robot is None:
            return
        if self.state != BACKTRACKING and self._trapped(robot):
            self._start_backtrack(robot)
            return
        if self.state == IDLE:
            if self.retry_after is None or self._now() >= self.retry_after:
                if not self._start_escape_if_trapped() and not self._visit_candidate(robot):
                    self._select_candidates(robot)
        elif self.state == ESCAPING:
            self._escape_step()
        elif self.state == PLANNING:
            self._check_plan_timeout()
        elif self.state == NAVIGATING:
            if self._listen_due():
                self._start_listening()
            else:
                self._monitor_goal(robot)
        elif self.state == LISTENING:
            if self._elapsed(self.listen_start) >= self.p['listen_duration']:
                self.listen_last = self._now()
                self._send_goal(self.goal)
        elif self.state == LOOKING:
            if self._elapsed(self.look_start) >= self.p['candidate_look_time']:
                self.state = IDLE

    # ------------------------------------------------------------------ candidatos a víctima
    def _on_candidates(self, msg: VictimDetectionArray):
        self.victim_candidates = [(v.position.x, v.position.y) for v in msg.victims]

    def _goal_masks(self):
        """Máscaras del /map (libre, ocupado, desconocido), holgura y costo aceptable."""
        grid, info = self._grid(self.map_msg)
        free, occupied, unknown = fc.classify(
            grid, self.p['free_threshold'], self.p['occupied_threshold'])
        clearance = fc.clearance_map(occupied, info.resolution)
        cost_ok = None
        if self.costmap_msg is not None:
            cost, cost_info = self._grid(self.costmap_msg)
            cost_ok = fc.align_to(info, cost, cost_info, fill=-1) <= self.p['max_goal_cost']
        return info, free, occupied, unknown, clearance, cost_ok

    def _near(self, point, places, radius) -> bool:
        return any(math.hypot(point[0] - x, point[1] - y) < radius for x, y in places)

    def _pending_candidates(self):
        """Candidatos aún sin visitar y fuera de las zonas de encierro."""
        r = self.p['candidate_done_radius']
        return [c for c in self.victim_candidates
                if not self._near(c, self.visited_candidates, r)
                and not any(math.hypot(c[0] - zx, c[1] - zy) < zr
                            for zx, zy, zr in self.trap_zones)]

    def _visit_candidate(self, robot) -> bool:
        """
        Envía al robot a mirar el candidato pendiente más cercano; False si no hay ninguno.

        Un detector guía (sonido) sabe más o menos dónde hay alguien pero no lo confirma: el
        robot se acerca a un punto libre a candidate_view_min–max m con línea de vista y se
        queda mirándolo candidate_look_time s para que la cámara (forma, color) lo confirme.
        Cada candidato se visita una sola vez, se confirme o no.
        """
        if not self.p['candidate_visits_enabled'] or \
                self.candidate_visits >= self.p['candidate_max_visits']:
            return False
        pending = self._pending_candidates()
        if not pending:
            return False
        target = min(pending, key=lambda c: math.hypot(c[0] - robot[0], c[1] - robot[1]))
        info, free, occupied, _, clearance, cost_ok = self._goal_masks()
        view = fc.find_view_goal(target, robot, free, occupied, clearance, info,
                                 self.p['candidate_view_min'], self.p['candidate_view_max'],
                                 self.p['min_goal_clearance'], cost_ok)
        self.visited_candidates.append(target)
        if view is None:
            self.get_logger().info(f'Candidato en ({target[0]:.2f}, {target[1]:.2f}): sin punto '
                                   'libre desde el que mirarlo; se descarta')
            return False
        self.candidate_visits += 1
        goal = fc.Frontier(cells=np.zeros((0, 2), dtype=int), centroid=target, goal=view[:2])
        goal.extra.update({'candidate': target, 'yaw': view[2]})
        self.goals_sent += 1
        self.get_logger().info(
            f'Meta {self.goals_sent}: mirar el candidato a víctima {self.candidate_visits} en '
            f'({target[0]:.2f}, {target[1]:.2f}) desde ({view[0]:.2f}, {view[1]:.2f})')
        self._send_goal(goal)
        return True

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
        rejected = {'sin_holgura': 0, 'cerca': 0, 'poca_info': 0, 'lista_negra': 0, 'trampa': 0}
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
                                                         self.p['blacklist_radius']) else
                      'trampa' if any(math.hypot(f.goal[0] - zx, f.goal[1] - zy) < zr
                                      for zx, zy, zr in self.trap_zones) else None)
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
        fc.score_frontiers(self.candidates, self.p['alpha'], self.p['beta'], self.p['gamma'],
                           self.victim_candidates, self.p['sign_sigma'])
        best = fc.best_frontier(self.candidates)
        if best is None:
            self.state = IDLE
            return
        self.goals_sent += 1
        self.get_logger().info(
            f'Meta {self.goals_sent}: ({best.goal[0]:.2f}, {best.goal[1]:.2f}) '
            f'J={best.score:.3f} C={best.path_cost:.2f} m I={best.info_gain:.0f} celdas '
            + (f'S={best.extra.get("sign", 0.0):.2f} ' if self.p['gamma'] > 0 else '')
            + f'({len(self.candidates)} candidatos, {len(self.frontiers)} fronteras)')
        self._send_goal(best)
        self._publish_markers()

    def _send_goal(self, best):
        """Envía (o reenvía tras una parada de escucha) la meta a NavigateToPose."""
        self.goal = best
        self.goal_seq += 1
        seq = self.goal_seq
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(*best.goal, yaw=best.extra.get('yaw'))
        self.state = NAVIGATING
        self.goal_start = self.last_replan = self.progress_time = self._now()
        self.progress_pose = self._robot_xy()
        self.goal_handle = None
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda fut: self._on_nav_accepted(fut, seq))

    # ------------------------------------------------------------------ paradas de escucha
    def _listen_due(self) -> bool:
        """Cierto si toca detenerse a escuchar (listen_period s navegando desde la última)."""
        return bool(self.p['listen_enabled']) and self.p['listen_duration'] > 0 and \
            self._elapsed(self.listen_last) >= self.p['listen_period']

    def _start_listening(self):
        """
        Detiene el robot listen_duration s sin abandonar la meta.

        Con los motores en marcha el ruido tapa los sonidos débiles (y el radar Doppler solo ve
        la respiración con el robot quieto). Se cancela la meta en Nav2 (el controlador frena) y
        se reenvía la misma al terminar la parada.
        """
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_seq += 1            # invalida los callbacks de la meta cancelada
        self.listens += 1
        self.listen_start = self._now()
        self.escape_pub.publish(Twist())
        self.state = LISTENING
        self.get_logger().info(f'Parada de escucha {self.listens} '
                               f'({self.p["listen_duration"]:.0f} s)')

    # ------------------------------------------------------------------ ejecución de la meta
    def _on_nav_accepted(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:
            # La meta se abandonó mientras Nav2 la aceptaba (p. ej. empezó un retroceso): que
            # Nav2 no siga conduciendo hacia ella.
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
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
        target = self.goal.extra.get('candidate')
        if target is not None:
            # Candidato resuelto mientras el robot iba (la cámara ya lo confirmó o el detector
            # guía lo descartó): no hace falta llegar.
            if not self._near(target, self.victim_candidates, self.p['candidate_done_radius']):
                self._cancel_goal(success=True, reason='candidato resuelto')
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
        self._update_stuck(success, reason)
        self.blacklist.append(self.goal.goal)
        self.get_logger().info(
            f'Meta {"completada" if success else "fallida"} ({reason}); '
            f'lista negra: {len(self.blacklist)}')
        looking = success and self.goal.extra.get('candidate') is not None \
            and reason != 'candidato resuelto'
        self.goal, self.goal_handle = None, None
        self.goal_seq += 1          # invalida callbacks pendientes de la meta anterior
        self.state = IDLE
        if looking:
            # Llegó frente al candidato: se queda quieto mirándolo.
            self.look_start = self._now()
            self.state = LOOKING

    # ------------------------------------------------------------------ escape
    def _update_stuck(self, success, reason):
        """Cuenta las metas seguidas fallidas sin progreso con el robot en el mismo sitio."""
        robot = self._robot_xy()
        if success or reason != 'sin progreso' or robot is None:
            self.stuck_count, self.stuck_anchor = 0, None
            return
        if (self.stuck_anchor is None or math.hypot(robot[0] - self.stuck_anchor[0],
                                                    robot[1] - self.stuck_anchor[1])
                > self.p['progress_distance']):
            self.stuck_anchor, self.stuck_count = robot, 1
        else:
            self.stuck_count += 1

    def _start_escape_if_trapped(self) -> bool:
        """
        Inicia la maniobra de escape si el robot está atrapado.

        Dos casos: (1) el centro del robot está en la zona inscrita del costmap global; desde
        ahí el planificador de Nav2 rechaza cualquier ruta ("Start occupied"), aunque
        físicamente haya espacio. (2) Las últimas `stuck_failures` metas fallaron sin progreso
        con el robot en el mismo sitio: el controlador ve una colisión en su trayectoria y no
        avanza hacia ninguna meta. Devuelve True si se inició la maniobra.
        """
        pose = self._robot_pose()
        if self.costmap_msg is None or pose is None:
            return False
        cost, info = self._grid(self.costmap_msg)
        inscribed = (fc.cost_at(cost, info, pose[0], pose[1], outside=0)
                     >= self.p['escape_cost_threshold'])
        stuck = self.stuck_count >= self.p['stuck_failures'] > 0
        if not inscribed and not stuck:
            return False
        self.escapes += 1
        self.stuck_count, self.stuck_anchor = 0, None
        cause = ('Robot en zona inscrita del costmap' if inscribed else
                 f'{self.p["stuck_failures"]} metas seguidas sin progreso en el mismo sitio')
        if self.escape_failures >= self.p['escape_bypass_after'] > 0:
            # Último recurso: los escapes anteriores no movieron el robot porque un obstáculo
            # quedó dentro del polígono del collision_monitor (que entonces frena todo, también
            # alejarse). Avance o retroceso recto y muy lento, sin girar, hacia el lado de menor
            # costo, publicado directamente en la base: el centro está a más de robot_radius
            # menos el margen del obstáculo y la carcasa no lo alcanza al alejarse en línea recta.
            forward = fc.straight_escape_forward(cost, info, *pose,
                                                 self.p['escape_probe_distances'])
            self.escape_failures = 0
            self.escape = {'heading': pose[2], 'forward': forward, 'start': pose[:2],
                           'since': self._now(), 'turning': False, 'inscribed': inscribed,
                           'bypass': True}
            self.get_logger().warn(
                f'{cause}: escape {self.escapes} DE ÚLTIMO RECURSO (los anteriores no movieron '
                f'el robot), {"avanzando" if forward else "retrocediendo"} en línea recta')
        else:
            heading, forward = fc.escape_direction(cost, info, *pose,
                                                   self.p['escape_probe_distances'])
            self.escape = {'heading': heading if forward else heading + math.pi,
                           'forward': forward, 'start': pose[:2], 'since': self._now(),
                           'turning': True, 'inscribed': inscribed, 'bypass': False}
            self.get_logger().info(
                f'{cause}: escape {self.escapes} hacia '
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
                # Si empezó en la zona inscrita, basta con salir de ella; si no, recorre
                # escape_distance completo.
                left_zone = esc['inscribed'] and free and moved > 0.05
                distance = self.p['escape_bypass_distance' if esc['bypass']
                                  else 'escape_distance']
                if moved >= distance or left_zone:
                    done = True
                else:
                    speed = self.p['escape_bypass_speed' if esc['bypass'] else 'escape_speed']
                    cmd.linear.x = speed if esc['forward'] else -speed
                    cmd.angular.z = 0.0 if esc['bypass'] else 1.5 * err
        publisher = self.bypass_pub if esc['bypass'] else self.escape_pub
        publisher.publish(Twist() if done else cmd)
        if done:
            moved = 0.0 if pose is None else math.hypot(pose[0] - esc['start'][0],
                                                        pose[1] - esc['start'][1])
            if esc['bypass'] or moved >= self.p['escape_min_motion']:
                self.escape_failures = 0
            else:
                self.escape_failures += 1
            self.escape = None
            self.state = IDLE

    # ------------------------------------------------------------------ retroceso por la huella
    def _control(self):
        """Ciclo rápido: registra la huella del recorrido y conduce el retroceso."""
        if self.state in (WAITING, FINISHED):
            return
        pose = self._robot_pose()
        if pose is None:
            return
        if self.state == BACKTRACKING:
            self._backtrack_step(pose)
        else:
            self.crumbs.add(pose[0], pose[1])

    def _trapped(self, robot) -> bool:
        """
        Cierto si el robot lleva trap_timeout s sin alejarse trap_radius del mismo sitio.

        Las metas fallidas y los escapes cortos no lo sacan de un hueco cuyas salidas el costmap
        ve cerradas (p. ej. entre una víctima, un escombro bajo y escombros altos).
        """
        if self.trap_anchor is None or math.hypot(
                robot[0] - self.trap_anchor[0], robot[1] - self.trap_anchor[1]) \
                > self.p['trap_radius']:
            self.trap_anchor, self.trap_since = robot, self._now()
            return False
        return self.p['trap_timeout'] > 0 and \
            self._elapsed(self.trap_since) > self.p['trap_timeout']

    def _start_backtrack(self, robot):
        """
        Cancela lo que esté haciendo y vuelve por su propia huella fuera del hueco.

        Si vuelve a quedar encerrado cerca de un encierro anterior, el retroceso y la zona
        evitada crecen (×2, ×3, ...): una zona evitada pequeña no impide que la ruta hacia otra
        meta lo vuelva a llevar por el mismo hueco.
        """
        repeats = sum(1 for zx, zy, zr in self.trap_zones
                      if math.hypot(robot[0] - zx, robot[1] - zy) < zr)
        scale = 1 + repeats
        path = self.crumbs.path_back(robot[0], robot[1],
                                     scale * self.p['backtrack_distance'],
                                     scale * self.p['trap_radius'])
        self.trap_zones.append((robot[0], robot[1], scale * self.p['trap_avoid_radius']))
        self.trap_anchor, self.trap_since = robot, self._now()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.goal is not None:
            self._goal_done(False, 'robot encerrado')
        if self.escape is not None:
            (self.bypass_pub if self.escape['bypass'] else self.escape_pub).publish(Twist())
            self.escape = None
        if len(path) < 2:
            self.get_logger().warn('Robot encerrado pero sin huella para volver; se evita la zona')
            self.state = IDLE
            return
        self.backtracks += 1
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))
        self.get_logger().warn(
            f'Robot encerrado en ({robot[0]:.2f}, {robot[1]:.2f}) '
            f'{self.p["trap_timeout"]:.0f} s en un radio de '
            f'{self.p["trap_radius"]} m: retroceso {self.backtracks} por su huella '
            f'({length:.1f} m); se evitan metas a menos de '
            f'{scale * self.p["trap_avoid_radius"]:.1f} m'
            + (f' (encierro repetido ×{scale})' if repeats else ''))
        self.backtrack = {'path': path, 'since': self._now(), 'last_pose': robot,
                          'last_motion': self._now()}
        self.state = BACKTRACKING

    def _backtrack_step(self, pose):
        bt = self.backtrack
        cmd = Twist()
        end = bt['path'][-1]
        reached = math.hypot(pose[0] - end[0], pose[1] - end[1]) < 0.15
        if math.hypot(pose[0] - bt['last_pose'][0], pose[1] - bt['last_pose'][1]) > 0.05:
            bt['last_pose'], bt['last_motion'] = pose[:2], self._now()
        timeout = self._elapsed(bt['since']) > self.p['backtrack_timeout']
        stalled = self._elapsed(bt['last_motion']) > self.p['backtrack_stall_time']
        if not (reached or timeout or stalled):
            v, w, nearest = fc.follow_path_command(
                pose, bt['path'], self.p['backtrack_lookahead'], self.p['backtrack_speed'],
                self.p['backtrack_turn_speed'])
            bt['path'] = bt['path'][nearest:] or bt['path'][-1:]
            cmd.linear.x, cmd.angular.z = v, w
            self.escape_pub.publish(cmd)
            return
        self.escape_pub.publish(Twist())
        result = 'fuera del hueco' if reached else \
            'tiempo agotado' if timeout else 'bloqueado por el collision_monitor'
        self.get_logger().info(f'Retroceso {self.backtracks} terminado ({result})')
        self.backtrack = None
        self.trap_anchor, self.trap_since = pose[:2], self._now()
        self.escape_failures = 0
        self.stuck_count, self.stuck_anchor = 0, None
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
            f'{self.goals_sent} metas enviadas, {self.goals_failed} fallidas, '
            f'{self.escapes} escapes, {self.backtracks} retrocesos, '
            f'{self.listens} paradas de escucha, {self.candidate_visits} candidatos visitados')
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
    except RuntimeError:
        # Carrera de rclpy al cerrar con Ctrl+C (espera o lee con el contexto ya cerrado);
        # cualquier otro RuntimeError se propaga.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
