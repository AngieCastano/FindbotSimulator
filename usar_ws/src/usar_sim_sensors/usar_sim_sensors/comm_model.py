"""
Modelo de la señal WiFi 2.4 GHz entre la estación remota y el robot, sin dependencias de ROS.

Gazebo no simula radio. Este modelo (Capa 2, excepción del ground truth: conoce la pose real
del robot y los escombros) calcula la potencia que mediría la radio:

    RSSI = P_tx + G_tx + G_rx − PL(d0) − 10·n·log10(d / d0) − N_esc·A_esc − N_muro·A_muro + X_σ

- PL(d0): pérdida en espacio libre a d0 = 1 m (Friis: 20·log10(4π·d0·f / c) ≈ 40 dB a 2.4 GHz).
- n: exponente de pérdida del entorno (2 en espacio libre; mayor entre escombros).
- A_esc: atenuación de cada escombro que corta la línea estación–robot (y A_muro por el muro
  perimetral si la estación está fuera del área).
- X_σ: desvanecimiento lento (sombra) gaussiano en dB, correlacionado en el tiempo.

Solo produce la potencia recibida: nunca posiciones, distancias ni identificadores.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from usar_sim_sensors.sound_model import count_obstacles_between

SPEED_OF_LIGHT = 299792458.0


def free_space_loss_db(distance: float, frequency: float) -> float:
    """Pérdida en espacio libre de Friis (dB) a `distance` m y `frequency` Hz."""
    return 20.0 * math.log10(4.0 * math.pi * distance * frequency / SPEED_OF_LIGHT)


def outside(point: Tuple[float, float], bounds: Dict) -> bool:
    """Cierto si el punto está fuera del área delimitada por los muros."""
    return not (bounds['x_min'] <= point[0] <= bounds['x_max']
                and bounds['y_min'] <= point[1] <= bounds['y_max'])


class WifiLinkModel:
    """RSSI del enlace estación–robot."""

    def __init__(self, obstacles: Sequence[Dict], bounds: Optional[Dict], params: Dict,
                 rng: np.random.Generator):
        self.obstacles = list(obstacles)
        self.bounds = bounds
        self.p = params
        self.rng = rng
        self.shadow = 0.0

    def mean_rssi(self, station: Tuple[float, float], robot: Tuple[float, float]
                  ) -> Tuple[float, int, int]:
        """(RSSI medio dBm, escombros en la línea, muros en la línea)."""
        p = self.p
        d = max(math.hypot(robot[0] - station[0], robot[1] - station[1]), p['ref_distance'])
        loss = free_space_loss_db(p['ref_distance'], p['frequency']) \
            + 10.0 * p['path_loss_exponent'] * math.log10(d / p['ref_distance'])
        n_obs = count_obstacles_between(station, robot, self.obstacles)
        n_walls = 0
        bounds = self.bounds
        if bounds is not None and outside(station, bounds) != outside(robot, bounds):
            n_walls = 1
        loss += n_obs * p['obstacle_attenuation_db'] + n_walls * p['wall_attenuation_db']
        rssi = p['tx_power_dbm'] + p['tx_gain_dbi'] + p['rx_gain_dbi'] - loss
        return rssi, n_obs, n_walls

    def measure(self, station: Tuple[float, float], robot: Tuple[float, float],
                dt: float) -> float:
        """RSSI medido (dBm) con la sombra correlacionada (Gauss-Markov de tiempo τ)."""
        p = self.p
        rssi, _, _ = self.mean_rssi(station, robot)
        if p['shadowing_sigma_db'] > 0:
            a = math.exp(-dt / max(p['shadowing_time'], 1e-6))
            self.shadow = a * self.shadow + math.sqrt(1 - a * a) * float(
                self.rng.normal(0.0, p['shadowing_sigma_db']))
        return rssi + self.shadow


DEFAULTS = {
    'frequency': 2.437e9,
    'ref_distance': 1.0,
    'tx_power_dbm': 20.0,
    'tx_gain_dbi': 2.0,
    'rx_gain_dbi': 2.0,
    'path_loss_exponent': 3.0,
    'obstacle_attenuation_db': 10.0,
    'wall_attenuation_db': 12.0,
    'shadowing_sigma_db': 3.0,
    'shadowing_time': 2.0,
}


def model_params(overrides: Optional[Dict] = None) -> Dict:
    """Parámetros del modelo (valores por defecto + los dados)."""
    return {**DEFAULTS, **(overrides or {})}
