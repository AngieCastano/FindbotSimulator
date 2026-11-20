"""Detector de calor corporal (Fase 9) y prolongación de rayos sobre el /map."""

import numpy as np
import pytest
from usar_detection.bearing_evidence import BearingEvidence
from usar_detection.map_filter import ray_extents
from usar_detection.sign_of_life import (create_detector, intrinsics_from_fov,
                                         ThermalObservation)
from usar_detection.thermal_detector import ThermalDetector

W, H = 32, 24
HFOV = 0.96
K = intrinsics_from_fov(W, H, HFOV)
CAM_HEIGHT = 0.117
# Base ← cámara óptica (x derecha, y abajo, z adelante) mirando a +x de la base.
BASE_CAM = np.array([[0.0, 0.0, 1.0, 0.0],
                     [-1.0, 0.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0, CAM_HEIGHT],
                     [0.0, 0.0, 0.0, 1.0]])


def render(boxes, ambient=20.0, sub=4):
    """
    Imagen térmica (°C) de cajas alineadas con los ejes sobre un suelo a temperatura ambiente.

    boxes: (x_min, x_max, y_min, y_max, z_max, temperatura) en el frame de la base. Cada píxel
    promedia sub×sub rayos (un píxel del MLX90640 integra todo lo que cubre).
    """
    off = (np.arange(sub) + 0.5) / sub
    v, u = np.mgrid[0:H, 0:W]
    out = np.zeros((H, W))
    o = BASE_CAM[:3, 3]
    for du in off:
        for dv in off:
            d_cam = np.stack([(u + du - K[2]) / K[0], (v + dv - K[5]) / K[4],
                              np.ones((H, W))], axis=-1)
            d = d_cam @ BASE_CAM[:3, :3].T
            best = np.full((H, W), np.inf)
            temp = np.full((H, W), ambient)
            with np.errstate(divide='ignore', invalid='ignore'):
                for x0, x1, y0, y1, z1, t in boxes:
                    t1 = (np.array([x0, y0, 0.0]) - o) / d
                    t2 = (np.array([x1, y1, z1]) - o) / d
                    near = np.nanmax(np.minimum(t1, t2), axis=-1)
                    far = np.nanmin(np.maximum(t1, t2), axis=-1)
                    hit = (near <= far) & (far > 0) & (near < best)
                    best = np.where(hit, near, best)
                    temp = np.where(hit, t, temp)
            out += temp
    return (out / sub ** 2).astype(np.float32)


def observe(image, base=True):
    return ThermalObservation(temperature=image, k=K, sensor_to_base=BASE_CAM if base else None)


# ---------------------------------------------------------------------- detector
def test_registered_as_thermal_sensor():
    det = create_detector('thermal', {})
    assert isinstance(det, ThermalDetector) and det.sensor == 'thermal'


def test_intrinsics_from_fov():
    fx = K[0]
    assert np.degrees(2 * np.arctan(W / 2 / fx)) == pytest.approx(55.0, abs=0.1)
    assert K[2] == 16.0 and K[5] == 12.0 and K[4] == fx


def test_victim_gives_rays_toward_it():
    # Víctima (34 °C) de lado a 2.5 m, centrada 0.5 m a la izquierda.
    img = render([(2.5, 3.1, -0.35, 1.35, 0.22, 34.0)])
    dets = create_detector('thermal', {}).detect(observe(img))
    assert len(dets) == 1
    det = dets[0]
    assert det.points is None and det.rays.shape[1] == 3
    base_rays = det.rays @ BASE_CAM[:3, :3].T
    bearings = np.degrees(np.arctan2(base_rays[:, 1], base_rays[:, 0]))
    expected = np.degrees([np.arctan2(-0.35, 2.5), np.arctan2(1.35, 2.5)])
    assert bearings.min() >= expected[0] - 2.0 and bearings.max() <= expected[1] + 2.0
    assert det.extra['temperature'] > 28.0
    assert np.all(det.ray_max_range >= 2.5)          # la cota nunca corta la distancia real


def test_floor_bound_close_victim():
    # A 1.5 m el píxel caliente más bajo queda bajo el horizonte: la cota descarta lo lejano.
    det = create_detector('thermal', {}).detect(
        observe(render([(1.5, 2.1, -0.85, 0.85, 0.22, 34.0)])))[0]
    assert np.all(det.ray_max_range >= 1.5)
    assert np.all(det.ray_max_range < 5.0)


def test_floor_only_and_noise_give_nothing():
    rng = np.random.default_rng(0)
    img = (np.full((H, W), 20.0) + rng.normal(0, 0.3, (H, W))).astype(np.float32)
    assert create_detector('thermal', {}).detect(observe(img)) == []


@pytest.mark.parametrize('temperature', [24.0, 60.0])
def test_lukewarm_and_too_hot_are_rejected(temperature):
    img = render([(2.0, 2.6, -0.85, 0.85, 0.22, temperature)])
    assert create_detector('thermal', {}).detect(observe(img)) == []


def test_single_hot_pixel_is_noise():
    img = np.full((H, W), 20.0, np.float32)
    img[12, 16] = 34.0
    assert create_detector('thermal', {}).detect(observe(img)) == []


def test_without_base_transform_uses_max_range():
    img = render([(2.5, 3.1, -0.85, 0.85, 0.22, 34.0)])
    det = create_detector('thermal', {'max_range': 5.0}).detect(observe(img, base=False))[0]
    assert np.all(det.ray_max_range == 5.0)


def test_debug_mask():
    det = create_detector('thermal', {})
    det.detect(observe(render([(2.5, 3.1, -0.85, 0.85, 0.22, 34.0)])))
    assert det.debug_mask().any()


# ---------------------------------------------------------------------- rayos sobre el /map
def _grid():
    """Mapa 5 m × 5 m (resolución 0.1, origen 0,0): libre, con una pared ocupada en x = 3 m."""
    data = np.zeros((50, 50), dtype=np.int8)
    data[:, 30] = 100
    data[40:, :] = -1                                # franja norte desconocida (y ≥ 4 m)
    return data.ravel()


def test_ray_extents():
    start = np.array([0.5, 1.0])
    dirs = np.array([[1.0, 0.0],        # hacia la pared (x = 3): hasta ella + 0.4 m
                     [0.0, 1.0],        # hacia el norte: hasta lo desconocido (y = 4)
                     [1.0, 0.0],        # hacia la pared, pero con distancia máxima 1 m
                     [-1.0, 0.0]])      # hacia el oeste: hasta el borde del mapa
    d = ray_extents(_grid(), 0.0, 0.0, 0.1, 50, 50, start, dirs, np.array([6, 6, 1, 6]),
                    beyond=0.4)
    assert d == pytest.approx([2.5 + 0.4, 3.0, 1.0, 0.5], abs=0.06)


def test_ray_extents_min_range():
    data = np.zeros((50, 50), dtype=np.int8)
    data[10, 5] = 100                                # obstáculo pegado al sensor: se ignora
    data[10, 20] = 100
    d = ray_extents(data.ravel(), 0.0, 0.0, 0.1, 50, 50, np.array([0.45, 1.05]),
                    np.array([[1.0, 0.0]]), np.array([5.0]), min_range=0.2)
    assert d[0] == pytest.approx(1.55, abs=0.06)


def _rays_to(target, origin, spread=0.0):
    """Rayo desde origin hacia target (desplazado `spread` m en perpendicular)."""
    d = np.asarray(target, float) - origin
    perp = np.array([-d[1], d[0]]) / np.linalg.norm(d)
    return (d + spread * perp)[None, :]


def test_triangulation_finds_source_seen_from_several_directions():
    ev = BearingEvidence(cell_size=0.15, n_bins=12, min_bearings=3)
    source = np.array([3.0, 3.0])
    for origin in ([0.0, 3.0], [3.0, 0.0], [0.5, 0.5]):     # tres puntos de vista (0°, 90°, 45°)
        origin = np.array(origin)
        d = _rays_to(source, origin)
        found = ev.add_rays(origin, d, [np.linalg.norm(source - origin) + 0.3])
    assert len(found) >= 1
    assert np.min(np.linalg.norm(found - source, axis=1)) < 0.2
    assert np.all(np.linalg.norm(found - source, axis=1) < 0.5)   # sin celdas lejos de la fuente


def test_triangulation_rejects_single_viewpoint():
    # El robot gira en el sitio mirando la misma mancha: siempre el mismo sector.
    ev = BearingEvidence(cell_size=0.15, n_bins=12, min_bearings=3)
    origin = np.array([0.0, 0.0])
    for _ in range(20):
        assert len(ev.add_rays(origin, np.array([[1.0, 0.1]]), [4.0])) == 0


def test_misranged_points_do_not_accumulate():
    # Rayos que pasan por encima de una víctima y terminan cada uno en un sitio distinto detrás
    # de ella: solo se cruzan cerca de la víctima, no en los puntos lejanos donde terminan.
    ev = BearingEvidence(cell_size=0.15, n_bins=12, min_bearings=3)
    victim = np.array([3.0, 3.0])
    ends = []
    for origin in ([0.0, 3.0], [3.0, 0.0], [0.5, 0.5]):
        origin = np.array(origin)
        d = _rays_to(victim, origin)
        length = np.linalg.norm(victim - origin) + 2.0           # 2 m más allá de la víctima
        ends.append(origin + d[0] / np.linalg.norm(d[0]) * length)
        found = ev.add_rays(origin, d, [length])
    for end in ends:
        assert ev.bearings(*end) == 1
    assert np.all(np.linalg.norm(found - victim, axis=1) < 0.5)
