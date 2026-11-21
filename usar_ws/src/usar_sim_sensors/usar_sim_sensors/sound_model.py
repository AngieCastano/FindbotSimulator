"""
Modelo de los sonidos de las víctimas y de la dirección de llegada que estima el ReSpeaker.

Gazebo no simula sonido. Este modelo (Capa 2, excepción del ground truth: conoce dónde están las
víctimas y los escombros) calcula qué entregaría el arreglo de micrófonos:

1. **Fuentes.** Cada víctima "vocal" pide ayuda o golpea en ráfagas: `burst_duration` s de sonido
   cada `burst_interval` s (sorteado por ráfaga, reproducible por semilla), con un nivel de
   `source_db` dB SPL a 1 m. Solo una fracción `vocal_fraction` de las víctimas hace ruido.
2. **Propagación.** Divergencia esférica (−20·log10 r) y `obstacle_attenuation_db` por cada
   escombro que corta la línea recta entre la víctima y los micrófonos.
3. **Ruido.** Ambiente (`ambient_db`) más los motores del robot (`motor_noise_db`) mientras se
   mueve: con el robot en marcha los sonidos débiles quedan tapados.
4. **Lo que entrega el arreglo** (como el procesador del ReSpeaker): una sola dirección, la del
   sonido más fuerte, si su relación señal/ruido supera `vad_snr_db` (detección de voz). El
   error de la dirección crece cuando la señal es débil, y con cierta probabilidad (mayor si hay
   escombros en medio) la dirección apunta a un eco.

Solo produce dirección, nivel y detección de voz: nunca posiciones ni distancias.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def segment_hits_box(p0, p1, cx, cy, sx, sy, yaw) -> bool:
    """Cierto si el segmento p0–p1 corta el rectángulo (centro, tamaño, giro)."""
    c, s = math.cos(-yaw), math.sin(-yaw)

    def local(p):
        dx, dy = p[0] - cx, p[1] - cy
        return c * dx - s * dy, s * dx + c * dy

    (x0, y0), (x1, y1) = local(p0), local(p1)
    t0, t1 = 0.0, 1.0
    for p, q in ((-(x1 - x0), x0 + sx / 2), (x1 - x0, sx / 2 - x0),
                 (-(y1 - y0), y0 + sy / 2), (y1 - y0, sy / 2 - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return False
    return True


def segment_hits_circle(p0, p1, cx, cy, radius) -> bool:
    """Cierto si el segmento p0–p1 pasa a menos de radius del centro."""
    d = np.subtract(p1, p0)
    f = np.subtract(p0, (cx, cy))
    length2 = float(d @ d)
    t = 0.0 if length2 < 1e-12 else min(1.0, max(0.0, -float(f @ d) / length2))
    closest = f + t * d
    return float(closest @ closest) < radius * radius


def count_obstacles_between(p0, p1, obstacles: Sequence[Dict]) -> int:
    """Escombros del ground truth que cortan la línea p0–p1."""
    n = 0
    for o in obstacles:
        if o['type'] == 'box':
            hit = segment_hits_box(p0, p1, o['x'], o['y'], o['size_x'], o['size_y'], o['yaw'])
        else:
            hit = segment_hits_circle(p0, p1, o['x'], o['y'], o['radius'])
        n += int(hit)
    return n


def db_sum(levels: Sequence[float]) -> float:
    """Suma energética de niveles en dB."""
    return 10.0 * math.log10(sum(10.0 ** (lv / 10.0) for lv in levels) + 1e-30)


class VictimSounds:
    """Calendario de ráfagas de sonido de cada víctima vocal (reproducible por semilla)."""

    def __init__(self, n_victims: int, vocal_fraction: float, burst_duration: float,
                 burst_interval: Tuple[float, float], rng: np.random.Generator):
        self.rng = rng
        self.duration = float(burst_duration)
        self.interval = tuple(float(v) for v in burst_interval)
        n_vocal = int(round(vocal_fraction * n_victims))
        self.vocal = {int(i) for i in rng.choice(n_victims, size=n_vocal, replace=False)} \
            if n_vocal else set()
        # Próximo inicio de ráfaga de cada víctima vocal (fase aleatoria inicial).
        self._next = {i: float(rng.uniform(0.0, self.interval[1])) for i in sorted(self.vocal)}

    def active(self, t: float) -> List[int]:
        """Víctimas que están emitiendo en el instante t (s)."""
        out = []
        for i in sorted(self._next):
            start = self._next[i]
            while t >= start + self.duration:          # ráfaga ya terminada: sortea la siguiente
                start += float(self.rng.uniform(*self.interval))
            self._next[i] = start
            if start <= t < start + self.duration:
                out.append(i)
        return out


class MicArrayModel:
    """Dirección de llegada, nivel y detección de voz que entregaría el arreglo."""

    def __init__(self, victims: Sequence[Tuple[float, float]], obstacles: Sequence[Dict],
                 params: Dict, rng: np.random.Generator):
        self.victims = [tuple(v) for v in victims]
        self.obstacles = list(obstacles)
        self.p = params
        self.rng = rng

    def noise_db(self, moving: bool) -> float:
        """Nivel de ruido de fondo (ambiente + motores si el robot se mueve)."""
        levels = [self.p['ambient_db']]
        if moving:
            levels.append(self.p['motor_noise_db'])
        return db_sum(levels)

    def received(self, mic_xy, sources: Sequence[int]) -> List[Tuple[int, float, int]]:
        """(víctima, nivel en los micrófonos dB, escombros en medio) de cada fuente activa."""
        out = []
        for i in sources:
            vx, vy = self.victims[i]
            r = max(math.hypot(vx - mic_xy[0], vy - mic_xy[1]), self.p['min_distance'])
            n = count_obstacles_between((vx, vy), mic_xy, self.obstacles)
            level = self.p['source_db'] - 20.0 * math.log10(r) \
                - n * self.p['obstacle_attenuation_db']
            out.append((i, level, n))
        return out

    def measure(self, mic_xy, mic_yaw: float, moving: bool, sources: Sequence[int]
                ) -> Tuple[float, float, bool, float]:
        """(azimut en el frame del arreglo, nivel total dB, voz, confianza)."""
        p = self.p
        noise = self.noise_db(moving)
        heard = self.received(mic_xy, sources)
        total = db_sum([noise] + [lv for _, lv, _ in heard])
        if not heard:
            return float(self.rng.uniform(-math.pi, math.pi)), total, False, 0.0
        i, level, n_obs = max(heard, key=lambda h: h[1])
        snr = level - noise
        if snr < p['vad_snr_db']:
            return float(self.rng.uniform(-math.pi, math.pi)), total, False, 0.0
        vx, vy = self.victims[i]
        bearing = math.atan2(vy - mic_xy[1], vx - mic_xy[0]) - mic_yaw
        echo_prob = min(p['echo_prob_max'], p['echo_prob'] + p['echo_prob_per_obstacle'] * n_obs)
        if self.rng.random() < echo_prob:
            azimuth = float(self.rng.uniform(-math.pi, math.pi))       # eco: dirección falsa
        else:
            sigma = math.radians(min(p['doa_sigma_max_deg'], max(
                p['doa_sigma_min_deg'],
                p['doa_sigma_deg_at_10db'] * 10.0 ** (-(snr - 10.0) / 20.0))))
            azimuth = bearing + float(self.rng.normal(0.0, sigma))
        azimuth = math.atan2(math.sin(azimuth), math.cos(azimuth))
        confidence = float(min(1.0, max(0.0, (snr - p['vad_snr_db']) / 20.0)))
        return azimuth, total, True, confidence


DEFAULTS = {
    'vocal_fraction': 1.0,
    'source_db': 65.0,
    'burst_duration': 2.0,
    'burst_interval': [10.0, 20.0],
    'obstacle_attenuation_db': 8.0,
    'min_distance': 0.3,
    'ambient_db': 35.0,
    'motor_noise_db': 55.0,
    'vad_snr_db': 6.0,
    'doa_sigma_deg_at_10db': 10.0,
    'doa_sigma_min_deg': 3.0,
    'doa_sigma_max_deg': 40.0,
    'echo_prob': 0.05,
    'echo_prob_per_obstacle': 0.1,
    'echo_prob_max': 0.6,
}


def model_params(overrides: Optional[Dict] = None) -> Dict:
    """Parámetros del modelo (valores por defecto + los dados)."""
    return {**DEFAULTS, **(overrides or {})}
