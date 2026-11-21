"""Detector de sonido (Fase 11): escucha solo con el robot quieto y triangula direcciones."""

import math

import numpy as np
from usar_detection.bearing_evidence import BearingEvidence
from usar_detection.map_filter import ray_extents
from usar_detection.sign_of_life import create_detector, SoundObservation
from usar_detection.sound_detector import SoundDetector  # noqa: F401  (registra el detector)


def obs(t, x=0.0, y=0.0, yaw=0.0, az=0.0, voice=True, conf=0.5):
    return SoundObservation(azimuth=az, level_db=50.0, voice_activity=voice, confidence=conf,
                            position=(x, y), yaw=yaw, stamp_sec=t)


def test_registered_and_ignores_unknown_and_rubble():
    det = create_detector('sound', {})
    assert det.sensor == 'sound'
    bp = det.bearing_params()
    assert bp['stop_unknown'] is False and bp['beyond'] > 100


def test_only_listens_when_stationary():
    det = create_detector('sound', {})
    # En marcha (0.2 m/s): nunca da rayos.
    assert all(det.detect(obs(k * 0.1, x=0.02 * k)) == [] for k in range(30))
    # Quieto: tras 0.8 s de ventana empieza a dar rayos.
    det = create_detector('sound', {})
    outs = [det.detect(obs(k * 0.1)) for k in range(20)]
    assert outs[3] == [] and len(outs[15]) == 1


def test_ray_in_global_frame_and_requires_voice():
    det = create_detector('sound', {})
    for k in range(15):
        out = det.detect(obs(k * 0.1, yaw=math.pi / 2, az=-math.pi / 2))
    ray = out[0].rays[0]
    assert abs(ray[0] - 1.0) < 1e-9                # robot hacia +y y sonido a −90°: +x
    assert abs(ray[1]) < 1e-9
    assert det.detect(obs(1.6, voice=False)) == []
    assert det.detect(obs(1.7, conf=0.0)) == []


def test_rays_cross_rubble_and_unknown():
    data = np.zeros((40, 40), dtype=np.int8)        # 4 m × 4 m, resolución 0.1
    data[:, 15:20] = 100                           # escombro entre x = 1.5 y 2 m...
    data[:, 16:19] = -1                            # ...con el interior sin cartografiar
    start, d = np.array([0.5, 2.0]), np.array([[1.0, 0.0]])
    blocked = ray_extents(data.ravel(), 0, 0, 0.1, 40, 40, start, d, np.array([10.0]))
    through = ray_extents(data.ravel(), 0, 0, 0.1, 40, 40, start, d, np.array([10.0]),
                          beyond=1e3, stop_unknown=False)
    assert blocked[0] < 1.2
    assert abs(through[0] - 3.5) < 0.06            # hasta el borde del mapa (x = 4 m)


def test_triangulation_behind_rubble():
    # Víctima detrás de un escombro, oída desde tres paradas: la intersección la ubica.
    ev = BearingEvidence(cell_size=0.3, n_bins=12, min_bearings=3)
    victim = np.array([4.0, 4.0])
    rng = np.random.default_rng(0)
    found = np.zeros((0, 2))
    for stop in ([0.0, 4.0], [4.0, 0.0], [0.5, 0.5], [7.5, 1.0]):
        stop = np.array(stop)
        for _ in range(10):
            ang = math.atan2(*(victim - stop)[::-1]) + rng.normal(0, math.radians(4))
            out = ev.add_rays(stop, np.array([[math.cos(ang), math.sin(ang)]]), [10.0], 0.3)
            if len(out):
                found = np.vstack([found, out])
    assert len(found) > 0
    center = found.mean(axis=0)
    assert np.hypot(*(center - victim)) < 0.6


def test_rays_from_one_spot_never_triangulate():
    # El robot quieto oye sonidos (y ecos) en todas direcciones desde el mismo sitio: las celdas
    # junto a él no deben contar como vistas desde varias direcciones.
    ev = BearingEvidence(cell_size=0.3, n_bins=12, min_bearings=3)
    origin = np.array([2.0, 2.0])
    for k in range(200):
        ang = 2 * math.pi * k / 37.0
        out = ev.add_rays(origin, np.array([[math.cos(ang), math.sin(ang)]]), [10.0], 0.3)
        assert len(out) == 0
    assert ev.min_marking_distance() > 1.0


def test_sound_alone_is_candidate_not_victim():
    # El sonido es un detector guía: solo da un candidato; con la forma en el mismo sitio, una
    # víctima (con ambas fuentes) y el candidato desaparece.
    from usar_detection.tracker import Track, VictimRegistry

    def conf(x, y, vid=1):
        t = Track(vid, (x, y, 0.1), 10, 0.0, 1.0)
        t.victim_id = vid
        return t

    reg = VictimRegistry(merge_distance=1.0)
    sigmas = {'human_shape': 0.3, 'sound': 0.7}
    newly, _ = reg.update({'sound': [conf(2.0, 0.0)], 'human_shape': []}, 5, sigmas, {'sound'})
    assert newly == [] and reg.victims == []
    assert [c.position[:2] for c in reg.candidates] == [(2.0, 0.0)]
    assert reg.candidates[0].sources == ['sound']
    newly, _ = reg.update({'sound': [conf(2.0, 0.0)], 'human_shape': [conf(2.5, 0.0)]}, 5,
                          sigmas, {'sound'})
    assert [v.sources for v in newly] == [['human_shape', 'sound']]
    assert newly[0].victim_id == 1 and reg.candidates == []


def test_cue_only_parameter():
    det = create_detector('sound', {'cue_only': True})
    assert det.cue_only
    assert not create_detector('human_shape', {}).cue_only
