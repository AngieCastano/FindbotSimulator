"""Utilidades compartidas por los tests de usar_worldgen."""

import os

import pytest
from usar_worldgen.cli import main as generate_main
from usar_worldgen.ground_truth import load_ground_truth


@pytest.fixture
def generate(tmp_path):
    """Ejecuta la CLI `generate` y devuelve (directorio, ground_truth)."""
    def _run(*args, name='world'):
        out = tmp_path / name
        assert generate_main([*args, '--out', str(out), '--quiet']) == 0
        return out, load_ground_truth(os.path.join(out, 'ground_truth.yaml'))
    return _run
