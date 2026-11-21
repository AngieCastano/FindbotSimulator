"""
Modelo de ruido de la odometría de ruedas (encoders del TT 1:48), sin dependencias de ROS.

Gazebo (DiffDrive) entrega la odometría que darían unos encoders perfectos: solo incluye el
patinaje real de las ruedas. Los encoders reales suman errores de cuantización, radio de rueda
y separación imprecisos y contacto irregular con el suelo. Se modelan como ruido gaussiano sobre
cada incremento de movimiento (Δs avance, Δθ giro), con varianza proporcional a lo recorrido
(paseo aleatorio, como el modelo de movimiento de odometría de Thrun et al.):

    σ_s² = k_ss·|Δs| + k_st·|Δθ|      [m²]
    σ_θ² = k_ts·|Δs| + k_tt·|Δθ|      [rad²]

Al ser la varianza proporcional (y no la desviación), el error acumulado no depende de la
frecuencia de publicación: tras recorrer L metros en línea recta, σ_s = sqrt(k_ss·L).
"""

import math

import numpy as np


def wrap_angle(angle):
    """Lleva un ángulo a (-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomNoiseModel:
    """Integra la odometría ruidosa a partir de las poses ideales de DiffDrive."""

    def __init__(self, k_ss, k_st, k_ts, k_tt, sigma_v, sigma_w, seed=0):
        self.k = (float(k_ss), float(k_st), float(k_ts), float(k_tt))
        self.sigma_v = float(sigma_v)
        self.sigma_w = float(sigma_w)
        self.rng = np.random.default_rng(seed)
        self.reset()

    def reset(self):
        """Olvida la pose ideal anterior y vuelve la odometría ruidosa al origen."""
        self.prev = None
        self.pose = [0.0, 0.0, 0.0]
        self.var_xy = 0.0
        self.var_yaw = 0.0

    def step(self, x, y, yaw, v, w):
        """
        Procesa una pose ideal (x, y, yaw) y su velocidad (v, w) en el marco odom.

        Devuelve (x, y, yaw, v, w) ruidosos. La primera llamada fija el origen común.
        """
        if self.prev is None:
            self.prev = (x, y, yaw)
            self.pose = [x, y, yaw]
            return x, y, yaw, v, w
        px, py, pyaw = self.prev
        self.prev = (x, y, yaw)
        # Incremento ideal en el marco del robot (avance, deslizamiento lateral, giro).
        dx, dy = x - px, y - py
        ds = math.cos(pyaw) * dx + math.sin(pyaw) * dy
        dl = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        dth = wrap_angle(yaw - pyaw)
        k_ss, k_st, k_ts, k_tt = self.k
        var_s = k_ss * abs(ds) + k_st * abs(dth)
        var_th = k_ts * abs(ds) + k_tt * abs(dth)
        ds += self.rng.normal(0.0, math.sqrt(var_s)) if var_s > 0 else 0.0
        dth += self.rng.normal(0.0, math.sqrt(var_th)) if var_th > 0 else 0.0
        self.var_xy += var_s
        self.var_yaw += var_th
        # El incremento está en el marco de la pose anterior: se gira con el rumbo ruidoso
        # anterior (sin ruido reproduce exactamente la odometría ideal).
        nx, ny, nyaw = self.pose
        nx += ds * math.cos(nyaw) - dl * math.sin(nyaw)
        ny += ds * math.sin(nyaw) + dl * math.cos(nyaw)
        self.pose = [nx, ny, wrap_angle(nyaw + dth)]
        v_n = v + self.rng.normal(0.0, self.sigma_v) if self.sigma_v > 0 else v
        w_n = w + self.rng.normal(0.0, self.sigma_w) if self.sigma_w > 0 else w
        return nx, ny, self.pose[2], v_n, w_n
