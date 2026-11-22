"""Pruebas de la fusión por puntaje de VictimRegistry (Fase 13)."""

import os

import pytest
from usar_detection.sign_of_life import create_detector
from usar_detection.tracker import Track, VictimRegistry
import yaml

WEIGHTS = {'human_shape': 1.0, 'thermal': 0.6, 'co2': 0.5, 'radar': 0.4, 'sound': 0.3}
SIGMAS = {'human_shape': 0.3, 'thermal': 0.5, 'co2': 1.0, 'radar': 1.5, 'sound': 0.7}


def _conf(x, y, vid=1):
    t = Track(vid, (x, y, 0.1), 10, 0.0, 1.0)
    t.victim_id = vid
    return t


def _update(reg, confirmed):
    return reg.update(confirmed, 5, SIGMAS, set(), WEIGHTS, 1.0)


def test_strong_detector_alone_confirms():
    reg = VictimRegistry(merge_distance=1.0)
    newly, _ = _update(reg, {'human_shape': [_conf(1.0, 1.0)]})
    assert len(newly) == 1 and newly[0].score == pytest.approx(1.0)
    assert reg.candidates == []


def test_weak_detector_alone_is_candidate():
    reg = VictimRegistry(merge_distance=1.0)
    newly, _ = _update(reg, {'thermal': [_conf(1.0, 1.0)]})
    assert newly == [] and reg.victims == []
    assert len(reg.candidates) == 1 and reg.candidates[0].sources == ['thermal']
    assert reg.candidates[0].score == pytest.approx(0.6)


def test_two_weak_detectors_together_confirm():
    reg = VictimRegistry(merge_distance=1.0)
    newly, _ = _update(reg, {'thermal': [_conf(1.0, 1.0)], 'co2': [_conf(1.6, 1.2)]})
    assert len(newly) == 1
    assert newly[0].sources == ['co2', 'thermal'] and newly[0].score == pytest.approx(1.1)
    # La posición se acerca a la del detector más preciso (calor, σ 0.5 frente a 1.0).
    assert abs(newly[0].position[0] - 1.0) < abs(newly[0].position[0] - 1.6)
    assert reg.candidates == []


def test_guides_alone_stay_candidates_and_far_detections_do_not_add():
    reg = VictimRegistry(merge_distance=1.0)
    _update(reg, {'radar': [_conf(0.0, 0.0)], 'sound': [_conf(0.5, 0.0)],
                  'thermal': [_conf(5.0, 5.0)]})
    assert reg.victims == []
    scores = sorted(round(c.score, 2) for c in reg.candidates)
    assert scores == [0.6, 0.7]


def test_compatibility_without_weights():
    """Sin pesos: los guía (cue_only) pesan 0 y los demás 1 (regla "O" anterior)."""
    reg = VictimRegistry(merge_distance=1.0)
    newly, _ = reg.update({'thermal': [_conf(1.0, 1.0)], 'sound': [_conf(4.0, 4.0)]}, 5,
                          SIGMAS, {'sound'})
    assert [v.sources for v in newly] == [['thermal']]
    assert [c.sources for c in reg.candidates] == [['sound']]


@pytest.mark.parametrize('name,overlay,expected', [
    ('human_shape', 'detector.yaml', 1.0), ('color_shape', 'detector.yaml', 1.0),
    ('thermal', 'detector_thermal.yaml', 0.6), ('co2', 'detector_co2.yaml', 0.5),
    ('sound', 'detector_sound.yaml', 0.3), ('radar', 'detector_radar.yaml', 0.4)])
def test_configured_weights(name, overlay, expected):
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', overlay)
    with open(path, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['victim_detector']['ros__parameters']
    assert params['confirm_score'] == 1.0 if 'confirm_score' in params else True
    det = create_detector(name, params[name])
    assert det.weight == pytest.approx(expected)
