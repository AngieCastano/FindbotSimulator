"""Pruebas del detector por color, la proyección 3D y el seguimiento de víctimas."""

import numpy as np
import pytest
from usar_detection import color_detector  # noqa: F401
from usar_detection.sign_of_life import (
    available_detectors, backproject, create_detector, Observation, SignOfLifeDetector)
from usar_detection.tracker import VictimTracker

K = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
ORANGE = (255, 102, 0)                   # color de las víctimas (1.0, 0.4, 0.0)
GREY = (120, 120, 120)


def scene(boxes, depth_value=2.0, size=(480, 640)):
    """Imagen gris con rectángulos naranjas y profundidad constante."""
    rgb = np.full((*size, 3), GREY, dtype=np.uint8)
    for r0, r1, c0, c1 in boxes:
        rgb[r0:r1, c0:c1] = ORANGE
    depth = np.full(size, depth_value, dtype=np.float32)
    return Observation(rgb=rgb, depth=depth, k=K)


def test_registry_and_interface():
    """El detector por color está registrado y cumple la interfaz común."""
    assert 'color_shape' in available_detectors()
    det = create_detector('color_shape', {})
    assert isinstance(det, SignOfLifeDetector)
    with pytest.raises(ValueError):
        create_detector('no_existe', {})


def test_backproject_center_and_offset():
    """El centro óptico proyecta sobre el eje z; un desplazamiento escala con la profundidad."""
    assert backproject(320, 240, 2.0, K) == pytest.approx((0.0, 0.0, 2.0))
    assert backproject(420, 140, 2.0, K) == pytest.approx((0.4, -0.4, 2.0))


def test_detects_orange_blob_and_estimates_3d_point():
    """Un blob naranja centrado en (420, 240) a 2 m queda 0.4 m a la derecha del eje óptico."""
    det = create_detector('color_shape', {})
    found = det.detect(scene([(220, 260, 400, 440)]))
    assert len(found) == 1
    assert found[0].pixel == pytest.approx((419.5, 239.5), abs=1.0)
    assert found[0].point == pytest.approx((0.4, 0.0, 2.0), abs=0.01)


def test_grey_scene_and_small_blobs_are_ignored():
    """Sin naranja no hay detecciones; un blob menor que min_blob_area se descarta."""
    det = create_detector('color_shape', {'min_blob_area': 150})
    assert det.detect(scene([])) == []
    assert det.detect(scene([(100, 108, 100, 108)])) == []      # 64 px


def test_two_separate_blobs_give_two_detections():
    """Dos víctimas separadas en la imagen producen dos detecciones."""
    det = create_detector('color_shape', {})
    found = det.detect(scene([(200, 240, 50, 120), (200, 240, 500, 580)]))
    assert len(found) == 2


def test_blob_without_valid_depth_is_rejected():
    """Si el blob está más allá de max_depth (o sin profundidad) no se usa."""
    det = create_detector('color_shape', {'max_depth': 6.0})
    obs = scene([(220, 260, 400, 440)], depth_value=np.inf)
    assert det.detect(obs) == []
    obs = scene([(220, 260, 400, 440)], depth_value=8.0)
    assert det.detect(obs) == []


def test_depth_median_ignores_background_pixels():
    """La mediana de profundidad del blob no se contamina con unos pocos píxeles de fondo."""
    det = create_detector('color_shape', {})
    obs = scene([(220, 260, 400, 440)], depth_value=2.0)
    obs.depth[220:225, 400:440] = 9.0                           # 12 % del blob, más lejos
    found = det.detect(obs)
    assert found[0].point[2] == pytest.approx(2.0)


def body(cx, cy, length=1.7, width=0.6, along_x=True, step=0.05):
    """Puntos en el suelo que cubren el rectángulo de una víctima acostada."""
    xs = np.arange(-length / 2, length / 2 + 1e-9, step)
    ys = np.arange(-width / 2, width / 2 + 1e-9, step)
    gx, gy = np.meshgrid(xs, ys)
    if not along_x:
        gx, gy = gy, gx
    return np.column_stack([cx + gx.ravel(), cy + gy.ravel(), np.full(gx.size, 0.1)])


def test_tracker_confirms_after_n_observations():
    """La víctima se confirma en la captura N y su posición es el centro del cuerpo."""
    tracker = VictimTracker(min_observations=3, cell_size=0.1, join_distance=0.2)
    pts = body(2.0, 1.0)
    assert tracker.update(pts, 0.0) == []
    assert tracker.update(pts, 0.2) == []
    confirmed = tracker.update(pts, 0.4)
    assert len(confirmed) == 1 and confirmed[0].victim_id == 1
    assert confirmed[0].position[:2] == pytest.approx((2.0, 1.0), abs=0.06)
    assert tracker.update(pts, 0.6) == []                         # no se reconfirma


def test_parts_seen_separately_are_one_victim():
    """Cabeza vista desde un lado y piernas desde otro forman una sola víctima centrada."""
    tracker = VictimTracker(min_observations=4, cell_size=0.1, join_distance=0.2)
    full = body(0.0, 0.0)
    head_side = full[full[:, 0] > 0.0]
    legs_side = full[full[:, 0] <= 0.0]
    for t in range(3):
        tracker.update(head_side, t)
        tracker.update(legs_side, t + 0.5)
    assert len(tracker.confirmed()) == 1
    assert tracker.confirmed()[0].position[:2] == pytest.approx((0.0, 0.0), abs=0.1)


def test_close_victims_stay_separate():
    """Dos víctimas con 0.5 m de suelo libre entre ellas no se fusionan."""
    tracker = VictimTracker(min_observations=2, cell_size=0.1, join_distance=0.2)
    pts = np.vstack([body(0.0, 0.0), body(0.0, 1.1)])            # bordes a 0.5 m
    for t in range(3):
        tracker.update(pts, t)
    assert [v.victim_id for v in tracker.confirmed()] == [1, 2]


def test_single_frame_noise_is_ignored():
    """Una celda vista en una sola captura no forma ni siquiera un grupo tentativo."""
    tracker = VictimTracker(min_observations=2, cell_size=0.1, min_cell_frames=2)
    tracker.update([(5.0, 5.0, 0.1)] * 50, 0.0)
    assert tracker.tracks == []


def test_victim_ids_are_stable():
    """Nuevas observaciones no cambian el número de una víctima ya confirmada."""
    tracker = VictimTracker(min_observations=2, cell_size=0.1)
    for t in range(2):
        tracker.update(body(0.0, 0.0), t)
    for t in range(2):
        tracker.update(np.vstack([body(0.0, 0.0), body(5.0, 0.0)]), 10 + t)
    victims = {v.victim_id: v.position[0] for v in tracker.confirmed()}
    assert victims[1] == pytest.approx(0.0, abs=0.1)
    assert victims[2] == pytest.approx(5.0, abs=0.1)


def test_confidence_grows_with_observations():
    """La confianza es 0.5 al confirmarse y se satura en 1.0."""
    tracker = VictimTracker(min_observations=4, cell_size=0.1)
    for t in range(4):
        tracker.update(body(0.0, 0.0), t)
    assert tracker.confirmed()[0].confidence(4) == pytest.approx(0.5)
    for t in range(10):
        tracker.update(body(0.0, 0.0), 10 + t)
    assert tracker.confirmed()[0].confidence(4) == 1.0


def test_blob_with_two_depths_splits_into_two_victims():
    """Un blob continuo en la imagen con dos víctimas a 3 m y 8 m da dos detecciones."""
    det = create_detector('color_shape', {'max_depth': 10.0})
    obs = scene([(200, 260, 200, 440)], depth_value=3.0)
    obs.depth[200:260, 320:440] = 8.0                           # mitad derecha, más lejos
    found = sorted(det.detect(obs), key=lambda d: d.point[2])
    assert len(found) == 2
    assert found[0].point[2] == pytest.approx(3.0) and found[1].point[2] == pytest.approx(8.0)
    assert found[0].pixel[0] < 320 < found[1].pixel[0]


def test_single_victim_seen_obliquely_stays_one_detection():
    """Una víctima vista en diagonal (profundidad continua de 2 a 3 m) sigue siendo una."""
    det = create_detector('color_shape', {})
    obs = scene([(200, 260, 200, 440)])
    obs.depth[200:260, 200:440] = np.linspace(2.0, 3.0, 240, dtype=np.float32)[None, :]
    assert len(det.detect(obs)) == 1
