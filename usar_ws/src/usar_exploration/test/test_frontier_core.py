"""Pruebas unitarias de la lógica de fronteras con rejillas sintéticas."""

import math

import numpy as np
import pytest
from usar_exploration import frontier_core as fc

INFO = fc.GridInfo(resolution=0.1, origin_x=0.0, origin_y=0.0, width=20, height=10)


def make_grid():
    """Mitad izquierda libre (x < 1 m), mitad derecha desconocida, con un muro arriba."""
    grid = np.full((10, 20), fc.UNKNOWN, dtype=np.int16)
    grid[:, :10] = 0
    grid[9, :10] = 100
    return grid


def test_classify_thresholds():
    """Las celdas se clasifican según los umbrales de ocupación."""
    grid = np.array([[-1, 0, 20, 21, 65, 100]], dtype=np.int16)
    free, occupied, unknown = fc.classify(grid, free_threshold=20, occupied_threshold=65)
    assert free.tolist() == [[False, True, True, False, False, False]]
    assert occupied.tolist() == [[False, False, False, False, True, True]]
    assert unknown.tolist() == [[True, False, False, False, False, False]]


def test_frontier_is_free_cell_next_to_unknown():
    """La frontera es la columna libre contigua a lo desconocido (sin la celda del muro)."""
    free, _, unknown = fc.classify(make_grid(), 20, 65)
    mask = fc.frontier_mask(free, unknown)
    assert np.argwhere(mask).tolist() == [[r, 9] for r in range(9)]


def test_diagonal_contact_is_not_frontier():
    """Solo cuenta la vecindad 4: una celda desconocida en diagonal no crea frontera."""
    grid = np.zeros((3, 3), dtype=np.int16)
    grid[0, 0] = fc.UNKNOWN
    free, _, unknown = fc.classify(grid, 20, 65)
    mask = fc.frontier_mask(free, unknown)
    assert mask[0, 1] and mask[1, 0] and not mask[1, 1]


def test_clusters_split_and_small_ones_are_discarded():
    """Dos grupos separados forman dos clusters; los menores que min_size se descartan."""
    mask = np.zeros((10, 10), dtype=bool)
    mask[1, 1:6] = True          # 5 celdas
    mask[8, 7:9] = True          # 2 celdas
    info = fc.GridInfo(0.1, 0.0, 0.0, 10, 10)
    assert len(fc.cluster_frontiers(mask, info, min_size=1)) == 2
    clusters = fc.cluster_frontiers(mask, info, min_size=3)
    assert len(clusters) == 1 and clusters[0].size == 5
    assert clusters[0].centroid == pytest.approx((0.35, 0.15))


def test_diagonal_cells_join_the_same_cluster():
    """La agrupación usa 8-conectividad."""
    mask = np.eye(5, dtype=bool)
    info = fc.GridInfo(0.1, 0.0, 0.0, 5, 5)
    assert len(fc.cluster_frontiers(mask, info, min_size=1)) == 1


def test_representative_cell_belongs_to_cluster():
    """Para un cluster en forma de L, la celda representativa pertenece al cluster."""
    mask = np.zeros((10, 10), dtype=bool)
    mask[0, 0:6] = True
    mask[0:6, 0] = True
    info = fc.GridInfo(0.1, 0.0, 0.0, 10, 10)
    f = fc.cluster_frontiers(mask, info, 1)[0]
    cell = fc.representative_cell(f, info)
    assert mask[cell]


def test_information_counts_unknown_cells_in_radius():
    """I(g) cuenta las celdas desconocidas dentro del disco."""
    unknown = np.zeros((11, 11), dtype=bool)
    unknown[5, 5] = unknown[5, 7] = unknown[0, 0] = True
    gain = fc.information_map(unknown, radius_cells=2)
    assert gain[5, 5] == 2           # (5,5) y (5,7); (0,0) queda fuera del radio
    assert gain[10, 10] == 0


def test_safe_goal_keeps_clearance_from_obstacles():
    """La meta segura se aleja del muro hasta cumplir la holgura mínima."""
    grid = make_grid()
    free, occupied, _ = fc.classify(grid, 20, 65)
    clearance = fc.clearance_map(occupied, INFO.resolution)
    cell = fc.find_safe_goal((8, 9), free, clearance, INFO, search_radius=0.5,
                             min_clearance=0.3)
    assert cell is not None and free[cell] and clearance[cell] >= 0.3
    assert cell == (6, 9)            # la más cercana con 3 celdas de distancia al muro


def test_safe_goal_none_when_everything_is_tight():
    """Sin celdas con holgura suficiente no hay meta."""
    grid = make_grid()
    free, occupied, _ = fc.classify(grid, 20, 65)
    clearance = fc.clearance_map(occupied, INFO.resolution)
    assert fc.find_safe_goal((8, 9), free, clearance, INFO, 0.5, min_clearance=5.0) is None


def test_safe_goal_respects_costmap_mask():
    """El costmap puede vetar celdas que el /map considera libres (p. ej. extremidades bajas)."""
    grid = np.zeros((10, 10), dtype=np.int16)
    info = fc.GridInfo(0.1, 0.0, 0.0, 10, 10)
    free, occupied, _ = fc.classify(grid, 20, 65)
    clearance = fc.clearance_map(occupied, 0.1)
    cost_ok = np.ones((10, 10), dtype=bool)
    cost_ok[3:8, 3:8] = False
    cell = fc.find_safe_goal((5, 5), free, clearance, info, 0.5, 0.0, cost_ok)
    assert cell is not None and cost_ok[cell]


def test_blacklist_radius():
    """Una meta cerca de una meta fallida queda vetada."""
    assert fc.is_blacklisted((1.0, 1.0), [(1.2, 1.0)], radius=0.3)
    assert not fc.is_blacklisted((1.0, 1.0), [(1.5, 1.0)], radius=0.3)


def test_score_prefers_informative_frontier_when_beta_dominates():
    """Con β alto gana la frontera con más información; con α alto, la más cercana."""
    near = fc.Frontier(cells=np.zeros((1, 2)), centroid=(0, 0), info_gain=10, path_cost=1.0)
    far = fc.Frontier(cells=np.zeros((1, 2)), centroid=(0, 0), info_gain=100, path_cost=5.0)
    fc.score_frontiers([near, far], alpha=1.0, beta=5.0)
    assert fc.best_frontier([near, far]) is far
    fc.score_frontiers([near, far], alpha=5.0, beta=1.0)
    assert fc.best_frontier([near, far]) is near
    assert near.score == pytest.approx(5.0 * 0.2 - 1.0 * 0.1)


def test_score_with_infinite_cost_is_never_chosen():
    """Un candidato sin costo finito no se elige."""
    a = fc.Frontier(cells=np.zeros((1, 2)), centroid=(0, 0), info_gain=1, path_cost=math.inf)
    b = fc.Frontier(cells=np.zeros((1, 2)), centroid=(0, 0), info_gain=1, path_cost=2.0)
    fc.score_frontiers([a, b], 1.0, 1.0)
    assert fc.best_frontier([a, b]) is b
    fc.score_frontiers([a], 1.0, 1.0)
    assert fc.best_frontier([a]) is None


def test_path_length():
    """Longitud de una polilínea."""
    assert fc.path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)
    assert fc.path_length([(1, 1)]) == 0.0


def test_align_costmap_with_offset_origin():
    """El costmap con otro origen se coloca en las celdas correctas del mapa."""
    map_info = fc.GridInfo(0.1, 0.0, 0.0, 5, 5)
    cost_info = fc.GridInfo(0.1, 0.2, 0.1, 2, 2)
    cost = np.array([[1, 2], [3, 4]], dtype=np.int16)
    out = fc.align_to(map_info, cost, cost_info, fill=-1)
    assert out[1, 2] == 1 and out[2, 3] == 4 and out[0, 0] == -1


def test_world_cell_roundtrip():
    """Conversión celda ↔ mundo."""
    x, y = INFO.cell_to_world(3, 7)
    assert INFO.world_to_cell(x, y) == (3, 7)
    assert INFO.world_to_cell(-1.0, 0.0) is None


def test_ring_frontier_is_split_into_segments():
    """Un anillo (borde del primer escaneo) se divide en tramos cuyo centroide está sobre él."""
    info = fc.GridInfo(0.05, -2.0, -2.0, 80, 80)
    yy, xx = np.mgrid[0:80, 0:80]
    radius = np.hypot(xx - 39.5, yy - 39.5)
    ring = (radius >= 19) & (radius < 20)              # anillo de ~1 m de radio
    whole = fc.cluster_frontiers(ring, info, min_size=8)
    assert len(whole) == 1 and np.hypot(*whole[0].centroid) < 0.1
    parts = fc.cluster_frontiers(ring, info, min_size=8, segment_size=0.75)
    assert len(parts) >= 4
    assert sum(p.size for p in parts) <= ring.sum()
    for part in parts:
        assert np.hypot(*part.centroid) > 0.6          # cada tramo queda lejos del centro


def test_escape_direction_points_away_from_obstacle():
    """Con un obstáculo al este, la salida apunta al oeste (y, mirando al este, retrocede)."""
    info = fc.GridInfo(0.05, -1.0, -1.0, 40, 40)
    cost = np.zeros((40, 40), dtype=np.int16)
    cost[:, 22:] = 100                       # obstáculo desde x = 0.1 m
    cost[:, 18:22] = 99                      # zona inscrita
    heading, forward = fc.escape_direction(cost, info, 0.0, 0.0, yaw=0.0,
                                           probe_distances=(0.15, 0.3, 0.45))
    assert abs(abs(heading) - math.pi) < 0.6 and not forward
    heading, forward = fc.escape_direction(cost, info, 0.0, 0.0, yaw=math.pi,
                                           probe_distances=(0.15, 0.3, 0.45))
    assert abs(abs(heading) - math.pi) < 0.6 and forward


def test_escape_prefers_front_when_tied():
    """Sin obstáculos, la salida es hacia el frente."""
    info = fc.GridInfo(0.05, -1.0, -1.0, 40, 40)
    cost = np.zeros((40, 40), dtype=np.int16)
    heading, forward = fc.escape_direction(cost, info, 0.0, 0.0, yaw=0.5,
                                           probe_distances=(0.15, 0.3))
    assert forward and abs(heading - 0.5) < 1e-6


def test_escape_prefers_straight_motion_without_turning():
    """Si retroceder en línea recta basta para salir, no se gira aunque haya otra mejor."""
    info = fc.GridInfo(0.05, -1.0, -1.0, 40, 40)
    cost = np.zeros((40, 40), dtype=np.int16)
    cost[:, 22:] = 100                       # obstáculo al este (delante del robot, yaw = 0)
    cost[:, 18:22] = 99
    cost[:2, :] = 40                         # el oeste exacto tiene algo de costo, pero libre
    cost[:, :10] = 40
    heading, forward = fc.escape_direction(cost, info, 0.0, 0.0, yaw=0.0,
                                           probe_distances=(0.15, 0.3, 0.45))
    assert not forward and abs(abs(heading) - math.pi) < 1e-6


def test_straight_escape_moves_away_from_obstacle():
    """Escape de último recurso: recto, hacia el lado opuesto al obstáculo, sin girar."""
    info = fc.GridInfo(0.05, -1.0, -1.0, 40, 40)
    cost = np.zeros((40, 40), dtype=np.int16)
    cost[:, 22:] = 100                       # obstáculo al este
    assert not fc.straight_escape_forward(cost, info, 0.0, 0.0, 0.0, (0.15, 0.3))
    assert fc.straight_escape_forward(cost, info, 0.0, 0.0, math.pi, (0.15, 0.3))
    free = np.zeros((40, 40), dtype=np.int16)
    assert fc.straight_escape_forward(free, info, 0.0, 0.0, 0.3, (0.15, 0.3))


# ---------------------------------------------------------------------- retroceso por la huella
def test_breadcrumbs_spacing_and_limit():
    crumbs = fc.Breadcrumbs(spacing=0.1, max_points=50)
    for k in range(1000):
        crumbs.add(k * 0.01, 0.0)                     # 10 m en pasos de 1 cm
    assert len(crumbs.points) == 50
    gaps = np.diff(np.asarray(crumbs.points)[:, 0])
    assert np.all(gaps >= 0.1 - 1e-9)


def test_path_back_retraces_entry():
    # Entró por x (0 → 3) y giró hacia y (3, 0 → 1): vuelve por donde vino.
    crumbs = fc.Breadcrumbs(spacing=0.1)
    for k in range(31):
        crumbs.add(k * 0.1, 0.0)
    for k in range(1, 11):
        crumbs.add(3.0, k * 0.1)
    path = crumbs.path_back(3.0, 1.0, distance=2.0, min_exit=1.0)
    assert path[0] == pytest.approx((3.0, 1.0))
    assert path[-1][1] == pytest.approx(0.0)
    assert math.hypot(path[-1][0] - 3.0, path[-1][1] - 1.0) >= 1.0
    assert all(p[1] >= 0 for p in path)              # recorre su huella, no atajos
    assert fc.Breadcrumbs().path_back(0, 0, 2.0) == []


def test_follow_path_reverses_when_target_behind():
    path = [(0.0, 0.0), (-0.5, 0.0), (-1.0, 0.0)]
    v, w, _ = fc.follow_path_command((0.0, 0.0, 0.0), path, 0.3, 0.1, 0.5)
    assert v < 0 and abs(w) < 1e-6                   # marcha atrás, sin girar
    v, w, _ = fc.follow_path_command((0.0, 0.0, math.pi), path, 0.3, 0.1, 0.5)
    assert v > 0                                      # mirando hacia el camino: adelante


def test_follow_path_turns_in_place_for_side_target():
    v, w, _ = fc.follow_path_command((0.0, 0.0, 0.0), [(0.0, 0.0), (0.0, 1.0)], 0.3, 0.1, 0.5)
    assert v == 0.0 and w > 0


def test_follow_path_drives_back_along_crumbs():
    """Simulación cinemática: el robot sigue la huella marcha atrás hasta el final."""
    path = [(3.0 - 0.1 * k, 0.05 * math.sin(k / 5)) for k in range(25)]
    x, y, yaw = 3.0, 0.0, 0.0
    for _ in range(400):
        v, w, nearest = fc.follow_path_command((x, y, yaw), path, 0.3, 0.1, 0.5)
        path = path[nearest:] or path[-1:]
        x += v * math.cos(yaw) * 0.1
        y += v * math.sin(yaw) * 0.1
        yaw += w * 0.1
        if math.hypot(x - path[-1][0], y - path[-1][1]) < 0.15:
            break
    assert math.hypot(x - 0.6, y - 0.05 * math.sin(24 / 5)) < 0.2


def _room(n=60, res=0.1):
    """Sala libre de n×n celdas con borde ocupado."""
    grid = np.zeros((n, n), dtype=np.int16)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    return grid, fc.GridInfo(res, 0.0, 0.0, n, n)


def test_line_of_sight():
    grid, info = _room()
    _, occupied, _ = fc.classify(grid, 20, 65)
    assert fc.line_of_sight(occupied, info, (1.0, 1.0), (4.0, 1.0))
    occupied[5:15, 25] = True                       # pared en x = 2.5 entre y = 0.5 y 1.5
    assert not fc.line_of_sight(occupied, info, (1.0, 1.0), (4.0, 1.0))
    # El final del segmento no se comprueba: un objetivo pegado a un escombro sigue visible.
    assert fc.line_of_sight(occupied, info, (1.0, 1.0), (2.6, 1.0), ignore_end=0.3)


def test_find_view_goal_ring_sight_and_heading():
    grid, info = _room()
    grid[10:50, 30] = 100                           # pared en x = 3.0 entre y = 1.0 y 5.0
    free, occupied, _ = fc.classify(grid, 20, 65)
    clearance = fc.clearance_map(occupied, info.resolution)
    target, robot = (4.5, 3.0), (1.0, 3.0)          # objetivo detrás de la pared
    view = fc.find_view_goal(target, robot, free, occupied, clearance, info, 0.8, 2.0, 0.3)
    assert view is not None
    x, y, yaw = view
    d = math.hypot(x - target[0], y - target[1])
    assert 0.8 <= d <= 2.0
    assert fc.line_of_sight(occupied, info, (x, y), target, 0.3)
    assert x > 3.0 or y < 1.0 or y > 5.0           # no se queda detrás de la pared
    assert yaw == pytest.approx(math.atan2(target[1] - y, target[0] - x))
    # Sin celdas libres alrededor: None.
    blocked = np.zeros_like(free)
    assert fc.find_view_goal(target, robot, blocked, occupied, clearance, info,
                             0.8, 2.0, 0.3) is None
