"""
Signo de vida "respiración" con un sensor de CO₂ puntual (Figaro TGS4161).

El sensor solo dice cuánto CO₂ hay donde está el robot: no da dirección. Con el recorrido del
robot se construye un mapa de concentración y se busca de dónde sale el exceso:

1. **Deshacer el retardo del sensor.** El TGS4161 responde como un sistema de primer orden de
   constante `response_tau` (dato del fabricante, decenas de segundos): la lectura va por detrás
   de la concentración real. Con una ventana de `smooth_window` s se ajusta una recta a log(ppm)
   y se estima la concentración real como exp(log c + τ · d(log c)/dt), en la posición media de
   la ventana.
2. **Mapa de concentración.** Las estimaciones se promedian en celdas de `grid_cell` m. La línea
   base es el percentil `baseline_percentile` de las celdas (el aire limpio de la mayor parte del
   recorrido), o `ambient_ppm` mientras hay pocas.
3. **Zonas con exceso.** Celdas más de `excess_threshold` ppm por encima de la base, agrupadas
   (`cluster_join` m).
4. **Ajuste de la fuente.** Para cada zona se busca, en una rejilla de `fit_step` m a menos de
   `fit_radius` m de su celda máxima, el punto q que mejor explica las celdas cercanas con el
   modelo de dispersión exceso = A · exp(−|p − q| / decay_length) (A por mínimos cuadrados).
   Como el robot solo mide sobre su recorrido, la fuente puede estar a un lado del camino: el
   ajuste usa celdas a ambos lados y delante y detrás para ubicarla.

Precisión esperada: del orden de 1 m (el sensor es lento y la pluma fluctúa).
"""

from collections import deque
from typing import Dict, List

import numpy as np
from scipy import ndimage

from .sign_of_life import Co2Observation, Detection, register_detector, SignOfLifeDetector

DEFAULTS = {
    'response_tau': 20.0,          # s, constante de tiempo del sensor (fabricante)
    'smooth_window': 4.0,          # s
    'ambient_ppm': 400.0,
    'baseline_percentile': 50.0,
    'min_baseline_cells': 30,
    'grid_cell': 0.25,             # m
    'excess_threshold': 250.0,     # ppm sobre la línea base
    'cluster_join': 2.0,           # m: el CO₂ no separa fuentes más cercanas
    'min_cells': 3,
    'fit_radius': 2.0,             # m alrededor de la celda máxima
    'fit_step': 0.1,               # m
    'decay_length': 1.0,           # m, modelo de dispersión supuesto
    'evidence_height': 0.1,        # m (solo visualización)
}


@register_detector
class Co2Detector(SignOfLifeDetector):
    """Mapa de concentración de CO₂ sobre el recorrido → posición estimada de las fuentes."""

    name = 'co2'
    sensor = 'co2'
    position_sigma = 1.0          # m: la fuente se ubica con error del orden de 1 m

    def __init__(self, params: Dict):
        super().__init__({**DEFAULTS, **params})
        self._window = deque()                     # (t, log c, x, y)
        self._cells: Dict[tuple, list] = {}        # celda → [suma, n]
        self.last_estimate = None

    def tracker_params(self) -> Dict[str, float]:
        """
        Mapa de evidencia propio del CO₂.

        La fuente estimada deriva ±1 m de una actualización a otra (pluma alargada y turbulenta):
        las estimaciones a menos de 1 m se unen en un solo grupo, cuya posición es el promedio de
        todas ellas, en vez de confirmar la misma víctima varias veces.
        """
        return {'cell_size': 0.25, 'join_distance': 1.0}

    # ------------------------------------------------------------------ paso 1
    def _estimate(self, obs: Co2Observation):
        """Concentración real estimada (deshaciendo el retardo) y posición de la ventana."""
        p = self.params
        self._window.append((obs.stamp_sec, np.log(max(obs.concentration, 1e-3)),
                             obs.position[0], obs.position[1]))
        while self._window and obs.stamp_sec - self._window[0][0] > p['smooth_window']:
            self._window.popleft()
        if len(self._window) < 3:
            return None
        w = np.array(self._window)
        t = w[:, 0] - w[:, 0].mean()
        if np.ptp(t) <= 0:
            return None
        slope, intercept = np.polyfit(t, w[:, 1], 1)
        log_c = intercept + p['response_tau'] * slope
        return float(np.exp(log_c)), float(w[:, 2].mean()), float(w[:, 3].mean())

    # ------------------------------------------------------------------ pasos 2–4
    def detect(self, observation: Co2Observation) -> List[Detection]:
        """Fuentes de CO₂ estimadas con todo lo medido hasta ahora (frame global)."""
        p = self.params
        est = self._estimate(observation)
        if est is None:
            return []
        conc, x, y = est
        self.last_estimate = conc
        cell = p['grid_cell']
        key = (int(np.floor(x / cell)), int(np.floor(y / cell)))
        acc = self._cells.setdefault(key, [0.0, 0])
        acc[0] += conc
        acc[1] += 1
        return self._sources()

    def baseline(self) -> float:
        """Línea base de CO₂ (ppm) estimada con las celdas medidas."""
        p = self.params
        if len(self._cells) < int(p['min_baseline_cells']):
            return float(p['ambient_ppm'])
        means = np.array([s / n for s, n in self._cells.values()])
        return float(np.percentile(means, p['baseline_percentile']))

    def _sources(self) -> List[Detection]:
        p = self.params
        cell = p['grid_cell']
        keys = np.array(list(self._cells.keys()))
        excess = np.array([s / n for s, n in self._cells.values()]) - self.baseline()
        hot = excess > p['excess_threshold']
        if hot.sum() < int(p['min_cells']):
            return []
        hot_keys = keys[hot]
        origin = hot_keys.min(axis=0)
        grid = np.zeros(hot_keys.max(axis=0) - origin + 1, dtype=bool)
        grid[tuple((hot_keys - origin).T)] = True
        radius = max(int(round(p['cluster_join'] / (2 * cell))), 1)
        labels, count = ndimage.label(ndimage.binary_dilation(grid, iterations=radius))
        hot_labels = labels[tuple((hot_keys - origin).T)]
        centers = (keys + 0.5) * cell
        detections = []
        for label in range(1, count + 1):
            members = np.nonzero(hot)[0][hot_labels == label]
            if len(members) < int(p['min_cells']):
                continue
            peak = centers[members[np.argmax(excess[members])]]
            q, amplitude = self._fit(peak, centers, excess)
            detections.append(Detection(
                kind=self.name, point=(float(q[0]), float(q[1]), p['evidence_height']),
                pixel=(0.0, 0.0),
                confidence=float(min(1.0, amplitude / (4.0 * p['excess_threshold']))),
                extra={'excess': float(amplitude), 'cells': float(len(members))},
                points=np.array([[q[0], q[1], p['evidence_height']]])))
        return detections

    def _fit(self, peak, centers, excess):
        """Punto q y amplitud A que mejor explican el exceso medido cerca de peak."""
        p = self.params
        near = np.hypot(*(centers - peak).T) <= p['fit_radius'] + 2 * p['decay_length']
        pts, e = centers[near], excess[near]
        offsets = np.arange(-p['fit_radius'], p['fit_radius'] + 1e-9, p['fit_step'])
        gx, gy = np.meshgrid(offsets, offsets)
        inside = np.hypot(gx, gy) <= p['fit_radius']
        cand = np.column_stack([gx[inside], gy[inside]]) + peak           # (Q, 2)
        r = np.linalg.norm(cand[:, None, :] - pts[None, :, :], axis=2)    # (Q, P)
        f = np.exp(-r / p['decay_length'])
        amp = np.maximum((f * e).sum(axis=1) / (f * f).sum(axis=1), 0.0)
        sse = ((e[None, :] - amp[:, None] * f) ** 2).sum(axis=1)
        best = int(np.argmin(sse))
        return cand[best], float(amp[best])
