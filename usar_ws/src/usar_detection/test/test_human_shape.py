"""Detector de forma humana (Fase 8), filtro de forma del mapa de evidencia y registro común."""

import numpy as np
import pytest
from usar_detection.human_shape_detector import HumanShapeDetector
from usar_detection.sign_of_life import create_detector, Observation
from usar_detection.tracker import shape_matches, Track, VictimRegistry, VictimTracker

W, H = 320, 240
K = [233.85, 0.0, 160.0, 0.0, 233.85, 120.0, 0.0, 0.0, 1.0]   # 69° de campo horizontal
CAM_HEIGHT = 0.12
# Base ← cámara óptica (x derecha, y abajo, z adelante) con la cámara mirando a +x de la base.
BASE_CAM = np.array([[0.0, 0.0, 1.0, 0.0],
                     [-1.0, 0.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0, CAM_HEIGHT],
                     [0.0, 0.0, 0.0, 1.0]])


def render_depth(boxes, max_range=10.0):
    """
    Profundidad (z óptica) de una escena: suelo z=0 más cajas alineadas con los ejes.

    boxes: lista de (x_min, x_max, y_min, y_max, z_max) en el frame de la base (z_min = 0).
    """
    v, u = np.mgrid[0:H, 0:W]
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    d_cam = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], axis=-1)
    d = d_cam @ BASE_CAM[:3, :3].T                   # dirección del rayo en la base (por z=1)
    o = BASE_CAM[:3, 3]
    t_best = np.full((H, W), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t_floor = -o[2] / d[..., 2]
        t_best = np.where((d[..., 2] < 0) & (t_floor > 0), t_floor, t_best)
        for x0, x1, y0, y1, z1 in boxes:
            lo = np.array([x0, y0, 0.0])
            hi = np.array([x1, y1, z1])
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            t_near = np.nanmax(np.minimum(t1, t2), axis=-1)
            t_far = np.nanmin(np.maximum(t1, t2), axis=-1)
            hit = (t_near <= t_far) & (t_far > 0)
            t_hit = np.where(t_near > 0, t_near, t_far)
            t_best = np.where(hit & (t_hit < t_best), t_hit, t_best)
    depth = t_best.astype(np.float32)               # el rayo tiene z óptica = 1: t = profundidad
    depth[depth > max_range] = np.inf
    return depth


def observe(boxes):
    depth = render_depth(boxes)
    return Observation(rgb=np.zeros((H, W, 3), np.uint8), depth=depth, k=K,
                       sensor_to_base=BASE_CAM)


def detector(**overrides):
    return create_detector('human_shape', {'stride': 1, **overrides})


def to_base(points):
    return points @ BASE_CAM[:3, :3].T + BASE_CAM[:3, 3]


# ---------------------------------------------------------------------- detección por imagen
def test_registered():
    assert isinstance(create_detector('human_shape', {}), HumanShapeDetector)


def test_low_object_detected_with_position_and_height():
    # Víctima de lado a 2.5 m: 0.6 de fondo × 1.7 de largo × 0.22 de alto.
    dets = detector().detect(observe([(2.5, 3.1, -0.85, 0.85, 0.22)]))
    assert len(dets) == 1
    base = to_base(dets[0].points)
    assert base[:, 0].min() == pytest.approx(2.5, abs=0.05)
    assert base[:, 2].max() == pytest.approx(0.22, abs=0.03)
    assert dets[0].extra['height'] < 0.32


def test_floor_only_gives_nothing():
    assert detector().detect(observe([])) == []


def test_tall_rubble_is_not_a_victim():
    # Escombro alto (0.9 m): su parte baja no debe tomarse por un objeto bajo.
    assert detector().detect(observe([(2.0, 3.0, -0.5, 0.5, 0.9)])) == []


def test_low_object_next_to_tall_rubble():
    # Víctima a 2.5 m y, separado, un escombro alto a la izquierda: solo la víctima.
    dets = detector().detect(observe([(2.5, 3.1, -0.8, 0.4, 0.22),
                                      (2.3, 3.3, 1.2, 2.2, 1.0)]))
    assert len(dets) == 1
    assert to_base(dets[0].points)[:, 1].max() < 0.6


def test_too_close_to_verify_is_ignored():
    # A 0.4 m la cámara solo ve hasta ≈ 0.32 m de altura: no se sabe si el objeto sigue hacia
    # arriba (podría ser la base de un escombro alto).
    assert detector().detect(observe([(0.4, 0.8, -0.3, 0.3, 0.2)])) == []


def test_without_base_transform_returns_nothing():
    obs = observe([(2.5, 3.1, -0.85, 0.85, 0.22)])
    obs.sensor_to_base = None
    assert detector().detect(obs) == []


def test_two_separate_low_objects():
    dets = detector().detect(observe([(2.5, 3.1, -1.6, -0.6, 0.2), (2.5, 3.1, 0.6, 1.6, 0.2)]))
    assert len(dets) == 2


def test_debug_mask_marks_detection():
    det = detector(stride=2)
    det.detect(observe([(2.5, 3.1, -0.85, 0.85, 0.22)]))
    assert det.debug_mask().any()


# ---------------------------------------------------------------------- filtro de forma
SHAPE = HumanShapeDetector({}).group_shape()


def _track(length, width, height, ratio):
    t = Track(1, (0.0, 0.0, 0.1), 10, 0.0, 1.0)
    t.length, t.width, t.height, t.profile_ratio = length, width, height, ratio
    return t


@pytest.mark.parametrize('dims,ok', [
    ((1.7, 0.6, 0.22, 1.5), True),     # víctima completa
    ((1.7, 0.2, 0.22, 1.5), True),     # víctima vista solo de lado
    ((0.6, 0.4, 0.20, 1.0), False),    # solo el torso: aún no
    ((1.8, 0.2, 0.22, 1.0), False),    # tablón: altura uniforme
    ((1.3, 1.2, 0.20, 1.1), False),    # losa: demasiado ancha
    ((1.6, 0.2, 0.10, 1.4), False),    # demasiado baja
    ((0.4, 0.4, 0.25, 1.0), False),    # bloque pequeño
])
def test_shape_filter(dims, ok):
    assert shape_matches(_track(*dims), SHAPE) is ok


def _body_points(cx, cy, n=400, seed=0):
    """Nube de una víctima acostada a lo largo de x: piernas 0.14 m, torso 0.2, cabeza 0.22."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-0.85, 0.85, n)
    y = rng.uniform(-0.25, 0.25, n)
    z = np.where(x < -0.1, 0.14, np.where(x < 0.6, 0.20, 0.22)) * rng.uniform(0.5, 1.0, n)
    z[np.argmax(x)] = 0.22
    z[np.argmin(x)] = 0.14
    return np.column_stack([x + cx, y + cy, z])


def test_tracker_confirms_body_with_shape_filter():
    tracker = VictimTracker(3, shape=SHAPE)
    for frame in range(4):
        tracker.update(_body_points(2.0, 1.0, seed=frame), float(frame))
    victims = tracker.confirmed()
    assert len(victims) == 1
    v = victims[0]
    assert v.length == pytest.approx(1.7, abs=0.2)
    assert v.height == pytest.approx(0.22, abs=0.01)
    assert v.profile_ratio > 1.25


def test_tracker_rejects_plank():
    rng = np.random.default_rng(1)
    tracker = VictimTracker(3, shape=SHAPE)
    for frame in range(5):
        pts = np.column_stack([rng.uniform(0, 1.8, 300), rng.uniform(0, 0.2, 300),
                               np.full(300, 0.2)])
        tracker.update(pts, float(frame))
    assert tracker.confirmed() == []
    assert len(tracker.tentative()) == 1


def test_merged_neighbours_stay_confirmed():
    # Dos víctimas cercanas confirmadas por separado; luego su evidencia se une (grupo más ancho
    # que el máximo). No deben retirarse: conservan su número y su posición.
    tracker = VictimTracker(3, shape=SHAPE)
    for frame in range(4):
        pts = np.vstack([_body_points(2.0, 1.0, seed=frame),
                         _body_points(2.0, 2.0, seed=9 + frame)])
        tracker.update(pts, float(frame))
    before = {v.victim_id: v.position[:2] for v in tracker.confirmed()}
    assert len(before) == 2
    rng = np.random.default_rng(5)
    for frame in range(4, 7):
        bridge = np.column_stack([rng.uniform(1.2, 2.8, 300), rng.uniform(1.2, 1.8, 300),
                                  np.full(300, 0.12)])
        tracker.update(bridge, float(frame))
    assert len([t for t in tracker.tracks if not t.confirmed]) == 1      # un solo grupo unido
    after = {v.victim_id: v.position[:2] for v in tracker.confirmed()}
    assert after.keys() == before.keys()
    for vid, xy in before.items():
        assert after[vid] == pytest.approx(xy, abs=0.05)


def test_confirmed_victim_retracted_if_profile_breaks():
    # Si la evidencia nueva contradice el perfil (altura uniforme), sí se retira.
    tracker = VictimTracker(3, shape=SHAPE)
    for frame in range(4):
        tracker.update(_body_points(2.0, 1.0, seed=frame), float(frame))
    assert len(tracker.confirmed()) == 1
    flat = np.column_stack([np.linspace(1.15, 2.85, 200), np.full(200, 1.0), np.full(200, 0.22)])
    for frame in range(4, 7):
        tracker.update(flat, float(frame))
    assert tracker.confirmed() == []


# ---------------------------------------------------------------------- registro común
def _confirmed(x, y, obs=10, vid=1):
    t = Track(vid, (x, y, 0.1), obs, 0.0, 1.0)
    t.victim_id = vid
    return t


def test_registry_merges_sources_and_keeps_ids():
    reg = VictimRegistry(merge_distance=1.0)
    newly, _ = reg.update({'color_shape': [_confirmed(0, 0)],
                           'human_shape': [_confirmed(0.3, 0), _confirmed(5, 5)]}, 5)
    assert len(newly) == 2
    first = [v for v in reg.victims if v.position[0] < 1][0]
    assert first.sources == ['color_shape', 'human_shape']
    ids = {round(v.position[0]): v.victim_id for v in reg.victims}
    newly, retracted = reg.update({'human_shape': [_confirmed(0.2, 0), _confirmed(5, 5)]}, 5)
    assert newly == [] and retracted == []
    assert {round(v.position[0]): v.victim_id for v in reg.victims} == ids


def test_registry_reports_retracted():
    reg = VictimRegistry()
    reg.update({'human_shape': [_confirmed(0, 0)]}, 5)
    newly, retracted = reg.update({'human_shape': []}, 5)
    assert newly == [] and retracted == [1]
    newly, _ = reg.update({'human_shape': [_confirmed(0.1, 0)]}, 5)
    assert [v.victim_id for v in newly] == [1]       # vuelve con el mismo número


def test_known_cells_mask():
    from usar_detection.map_filter import known_cells_mask
    data = np.full((10, 10), -1, dtype=np.int8)      # 1 m × 1 m, resolución 0.1, origen (0, 0)
    data[:, :5] = 0                                  # mitad oeste conocida (libre)
    data[0, 6] = 100                                 # una celda ocupada al este
    pts = np.array([[0.2, 0.5, 0.1], [0.8, 0.5, 0.1], [0.65, 0.05, 0.1], [-1.0, 0.5, 0.1],
                    [0.2, 2.0, 0.1]])
    mask = known_cells_mask(pts, data.ravel(), 0.0, 0.0, 0.1, 10, 10)
    assert mask.tolist() == [True, False, True, False, False]
    assert known_cells_mask(np.zeros((0, 3)), data.ravel(), 0, 0, 0.1, 10, 10).size == 0


def test_registry_does_not_merge_same_detector_and_weights_precision():
    # La forma ve dos víctimas a 1.5 m; una estimación de CO₂ entre ambas no las une y apenas
    # mueve la posición de la más cercana.
    reg = VictimRegistry(merge_distance=1.0)
    shape = [_confirmed(0.0, 0.0, obs=10, vid=1), _confirmed(1.5, 0.0, obs=10, vid=2)]
    co2 = [_confirmed(0.7, 0.0, obs=2000, vid=1)]
    reg.update({'human_shape': shape, 'co2': co2}, 5, {'human_shape': 0.3, 'co2': 1.0})
    assert len(reg.victims) == 2
    xs = sorted(v.position[0] for v in reg.victims)
    assert xs[0] == pytest.approx(0.0, abs=0.07) and xs[1] == pytest.approx(1.5, abs=0.07)
    merged = [v for v in reg.victims if 'co2' in v.sources]
    assert len(merged) == 1
