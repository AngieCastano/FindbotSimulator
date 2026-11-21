#!/usr/bin/env python3
"""Animaciones tipo osciloscopio para el video de la simulación del hardware.

Usa SOLO datos reales de ngspice (hardware/spice/results/*.dat) y, para el Bode, vuelve a
correr el análisis AC de spice/radar_amp.cir. Nada se dibuja a mano: si cambian las
simulaciones, se vuelve a correr este script.

Salida (hardware/docs/video_assets/):
  a1_buck_arranque.gif   5V_USB y VBAT apareciendo en pantalla (batería 7,4 V)
  a2_rizado_zoom.gif     zoom progresivo hasta el rizado de 5V_USB
  a3_motor_pwm.gif       corriente del motor al arrancar y rizado del PWM de 20 kHz
  a4_radar_bode.gif      barrido en frecuencia del filtro del radar (Bode)
  a5_montecarlo.gif      histograma de Monte Carlo llenándose corrida a corrida

Requisitos: Python 3 + Pillow, ngspice (solo para a4). Uso:
  python3 hardware/docs/video_assets/make_animations.py
Para convertir a MP4 (opcional, requiere ffmpeg):
  ffmpeg -i a1_buck_arranque.gif -movflags faststart -pix_fmt yuv420p a1_buck_arranque.mp4
"""
import math
import re
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HW = Path(__file__).resolve().parents[2]
SP = HW / 'spice' / 'results'
OUT = Path(__file__).resolve().parent

W, H = 1280, 720
PAD_L, PAD_R, PAD_T, PAD_B = 120, 50, 90, 90
BG = (8, 12, 10)
GRID = (30, 60, 40)
AXIS = (90, 140, 100)
TXT = (200, 230, 210)
TRACE = [(60, 255, 120), (255, 200, 40), (90, 180, 255)]
LIMIT = (255, 80, 80)
FPS = 25

FONT_PATH = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
MONO_PATH = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'


def font(size, mono=False):
    try:
        return ImageFont.truetype(MONO_PATH if mono else FONT_PATH, size)
    except OSError:
        return ImageFont.load_default()


F_TITLE, F_LBL, F_TICK = font(30), font(22), font(18, mono=True)


# ------------------------------------------------------------------ datos
def read_dat(path, ncols):
    """wrdata de ngspice: pares (escala, valor) por vector → [t, v1, v2, ...]."""
    out = [[] for _ in range(ncols + 1)]
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 2 * ncols:
                continue
            out[0].append(float(p[0]))
            for k in range(ncols):
                out[k + 1].append(float(p[2 * k + 1]))
    return out


def window(cols, t0, t1):
    idx = [i for i, t in enumerate(cols[0]) if t0 <= t <= t1]
    return [[c[i] for i in idx] for c in cols]


def decimate(xs, ys, n=1800):
    """Reduce puntos conservando mínimos y máximos (para que el rizado no desaparezca)."""
    if len(xs) <= n:
        return xs, ys
    step = len(xs) / (n / 2)
    ox, oy = [], []
    for k in range(int(n / 2)):
        a, b = int(k * step), int((k + 1) * step)
        seg = range(a, min(b, len(xs)))
        if not seg:
            continue
        imin = min(seg, key=lambda i: ys[i])
        imax = max(seg, key=lambda i: ys[i])
        for i in sorted((imin, imax)):
            ox.append(xs[i])
            oy.append(ys[i])
    return ox, oy


# ------------------------------------------------------------------ dibujo
class Scope:
    """Pantalla de osciloscopio con ejes lineales (o logarítmico en x)."""

    def __init__(self, xlim, ylim, xlabel, ylabel, title, xlog=False, xdiv=10, ydiv=8):
        self.xlim, self.ylim, self.xlog = xlim, ylim, xlog
        self.xlabel, self.ylabel, self.title = xlabel, ylabel, title
        self.xdiv, self.ydiv = xdiv, ydiv

    def px(self, x, y):
        x0, x1 = self.xlim
        if self.xlog:
            x, x0, x1 = math.log10(max(x, 1e-12)), math.log10(x0), math.log10(x1)
        y0, y1 = self.ylim
        fx = (x - x0) / (x1 - x0)
        fy = (y - y0) / (y1 - y0)
        return PAD_L + fx * (W - PAD_L - PAD_R), H - PAD_B - fy * (H - PAD_T - PAD_B)

    def frame(self, note=''):
        img = Image.new('RGB', (W, H), BG)
        d = ImageDraw.Draw(img)
        l, r, t, b = PAD_L, W - PAD_R, PAD_T, H - PAD_B
        for k in range(self.xdiv + 1):
            x = l + k * (r - l) / self.xdiv
            d.line([(x, t), (x, b)], fill=GRID, width=1)
        for k in range(self.ydiv + 1):
            y = t + k * (b - t) / self.ydiv
            d.line([(l, y), (r, y)], fill=GRID, width=1)
        d.rectangle([l, t, r, b], outline=AXIS, width=2)
        # marcas de los ejes
        for k in range(self.xdiv + 1):
            fx = k / self.xdiv
            if self.xlog:
                lo, hi = math.log10(self.xlim[0]), math.log10(self.xlim[1])
                v = 10 ** (lo + fx * (hi - lo))
                if abs(math.log10(v) - round(math.log10(v))) > 1e-6:
                    continue
            else:
                v = self.xlim[0] + fx * (self.xlim[1] - self.xlim[0])
            s = fmt(v)
            x = l + fx * (r - l)
            d.text((x, b + 8), s, font=F_TICK, fill=TXT, anchor='ma')
        for k in range(self.ydiv + 1):
            v = self.ylim[0] + k * (self.ylim[1] - self.ylim[0]) / self.ydiv
            y = b - k * (b - t) / self.ydiv
            d.text((l - 10, y), fmt(v), font=F_TICK, fill=TXT, anchor='rm')
        d.text((W / 2, 20), self.title, font=F_TITLE, fill=TXT, anchor='ma')
        d.text(((l + r) / 2, H - 30), self.xlabel, font=F_LBL, fill=TXT, anchor='ma')
        d.text((20, 58), self.ylabel, font=F_LBL, fill=TXT)
        if note:
            d.text((r, 58), note, font=F_LBL, fill=TRACE[1], anchor='ra')
        return img, d

    def hline(self, d, y, label):
        _, py = self.px(self.xlim[0], y)
        x = PAD_L
        while x < W - PAD_R:  # línea discontinua
            d.line([(x, py), (min(x + 14, W - PAD_R), py)], fill=LIMIT, width=2)
            x += 24
        d.text((W - PAD_R - 8, py - 6), label, font=F_TICK, fill=LIMIT, anchor='rb')

    def trace(self, d, xs, ys, color, width=3):
        pts = [self.px(x, y) for x, y in zip(xs, ys)]
        if len(pts) > 1:
            d.line(pts, fill=color, width=width, joint='curve')


def fmt(v):
    if v == 0:
        return '0'
    a = abs(v)
    if a >= 1000:
        return f'{v / 1000:g}k'
    if a >= 1:
        return f'{v:.4g}'.replace('.', ',')
    if a >= 0.01:
        return f'{v:.3g}'.replace('.', ',')
    return f'{v:.2g}'.replace('.', ',')


def legend(d, items):
    y = PAD_T + 12
    for name, color in items:
        d.line([(PAD_L + 16, y + 10), (PAD_L + 56, y + 10)], fill=color, width=4)
        d.text((PAD_L + 66, y), name, font=F_LBL, fill=TXT)
        y += 32


def save_gif(name, frames, hold=FPS * 2):
    frames = frames + [frames[-1]] * hold  # se queda quieta al final
    pal = [f.convert('P', palette=Image.ADAPTIVE, colors=64) for f in frames]
    pal[0].save(OUT / name, save_all=True, append_images=pal[1:], duration=int(1000 / FPS),
                loop=0, optimize=True)
    print(f'  {name}: {len(frames)} cuadros ({len(frames) / FPS:.1f} s)')


def sweep(scope, series, nframes, hlines=(), note='', items=None):
    """La señal aparece de izquierda a derecha, como un barrido de osciloscopio."""
    x0, x1 = scope.xlim
    frames = []
    for k in range(1, nframes + 1):
        frac = k / nframes
        if scope.xlog:
            xe = 10 ** (math.log10(x0) + frac * (math.log10(x1) - math.log10(x0)))
        else:
            xe = x0 + frac * (x1 - x0)
        img, d = scope.frame(note)
        for y, lab in hlines:
            scope.hline(d, y, lab)
        for (xs, ys, _), c in zip(series, TRACE):
            n = sum(1 for x in xs if x <= xe)
            scope.trace(d, xs[:n], ys[:n], c)
            if n:  # punto brillante del haz
                px, py = scope.px(xs[n - 1], ys[n - 1])
                d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(255, 255, 255))
        if items:
            legend(d, items)
        frames.append(img)
    return frames


# ------------------------------------------------------------------ animaciones
def a1_buck():
    t, vout, vbat, _ = read_dat(SP / 'power_supply_74.dat', 3)
    t, vout, vbat = window([t, vout, vbat], 0.5e-3, 3.2e-3)
    tm = [x * 1e3 for x in t]
    x1, y1 = decimate(tm, vout)
    x2, y2 = decimate(tm, vbat)
    sc = Scope((0.5, 3.2), (4.4, 8.0), 'tiempo [ms]', 'tensión [V]',
               'Alimentación: regulador de 5 V con batería de 7,4 V', xdiv=9, ydiv=9)
    frames = sweep(sc, [(x1, y1, ''), (x2, y2, '')], 90,
                   hlines=[(4.75, 'mínimo USB 4,75 V')], note='motores arrancan en 1 ms · USB máx. en 2 ms',
                   items=[('5V_USB (salida del regulador)', TRACE[0]), ('VBAT (batería)', TRACE[1])])
    save_gif('a1_buck_arranque.gif', frames)


def a2_ripple():
    t, vout, _, _ = read_dat(SP / 'power_supply_74.dat', 3)
    t, vout = window([t, vout], 0.5e-3, 3.2e-3)
    tm = [x * 1e3 for x in t]
    # zoom progresivo: de la vista completa a una ventana de 40 µs en régimen (0,80–0,84 ms)
    full = ((0.5, 3.2), (4.4, 5.6))
    zw = [x for x, ti in zip(vout, tm) if 0.80 <= ti <= 0.84]
    vmin, vmax = min(zw), max(zw)
    m = (vmax - vmin) * 0.6
    zoom = ((0.80, 0.84), (vmin - m, vmax + m))
    frames = []
    n = 75
    for k in range(n + 1):
        f = (1 - math.cos(math.pi * k / n)) / 2  # suave al empezar y al terminar
        # interpolación logarítmica del ancho para que el zoom se sienta continuo
        xl = (full[0][0] ** (1 - f)) * (zoom[0][0] ** f)
        xw = ((full[0][1] - full[0][0]) ** (1 - f)) * ((zoom[0][1] - zoom[0][0]) ** f)
        yw = ((full[1][1] - full[1][0]) ** (1 - f)) * ((zoom[1][1] - zoom[1][0]) ** f)
        yc = (sum(full[1]) / 2) * (1 - f) + (sum(zoom[1]) / 2) * f
        sc = Scope((xl, xl + xw), (yc - yw / 2, yc + yw / 2), 'tiempo [ms]', 'tensión [V]',
                   'Zoom al rizado de 5V_USB (batería de 7,4 V)', ydiv=8)
        idx = [i for i, x in enumerate(tm) if xl <= x <= xl + xw]
        xs, ys = decimate([tm[i] for i in idx], [vout[i] for i in idx])
        note = f'en esta ventana de 40 µs: {(vmax - vmin) * 1e3:.1f} mV pico a pico'.replace('.', ',')
        img, d = sc.frame(note if k == n else '')
        sc.trace(d, xs, ys, TRACE[0])
        frames.append(img)
    save_gif('a2_rizado_zoom.gif', frames, hold=FPS * 3)


def a3_motor():
    t, i, w, _ = read_dat(SP / 'motor_start.dat', 3)
    t1, i1 = window([t, i], 0, 0.4)
    x1, y1 = decimate(t1, i1)
    sc = Scope((0, 0.4), (0, 1.6), 'tiempo [s]', 'corriente [A]',
               'Motor: corriente al arrancar con PWM (duty máximo, 5,0 V)', ydiv=8)
    frames = sweep(sc, [(x1, y1, '')], 70, hlines=[(1.2, 'límite medio TB6612 1,2 A')],
                   items=[('corriente del motor', TRACE[0])])
    # luego zoom a 1 ms de régimen para ver el rizado del PWM de 20 kHz
    t2, i2 = window([t, i], 0.399, 0.4)
    tm = [x * 1e3 for x in t2]
    lo, hi = min(i2), max(i2)
    m = (hi - lo) * 0.8
    sc2 = Scope((399.0, 400.0), (lo - m, hi + m), 'tiempo [ms]', 'corriente [A]',
                'Zoom: rizado del PWM de 20 kHz (20 pulsos por ms)', ydiv=8)
    frames += sweep(sc2, [(tm, i2, '')], 50, note=f'rizado {(hi - lo) * 1e3:.0f} mA pico a pico',
                    items=[('corriente del motor', TRACE[0])])
    save_gif('a3_motor_pwm.gif', frames)


def a4_bode():
    """Corre el AC del radar con ngspice y anima el barrido de frecuencia."""
    src = (HW / 'spice' / 'radar_amp.cir').read_text()
    with tempfile.TemporaryDirectory() as tmp:
        dat = Path(tmp) / 'bode.dat'
        src = re.sub(r'(ac dec 100 0\.001 10k\n)', rf'\1wrdata {dat} vdb(adc)\n', src)
        src = re.sub(r'^hardcopy .*$', '', src, flags=re.M)  # no tocar las figuras del repo
        cir = HW / 'spice' / '.video_bode_tmp.cir'
        cir.write_text(src)
        try:
            subprocess.run(['ngspice', '-b', cir.name], cwd=HW / 'spice', check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        finally:
            cir.unlink(missing_ok=True)
        f, g = read_dat(dat, 1)
    keep = [k for k, v in enumerate(g) if v >= 0]  # el eje empieza en 0 dB
    f, g = [f[k] for k in keep], [g[k] for k in keep]
    gmax = max(g)
    sc = Scope((0.001, 10000), (0, 75), 'frecuencia [Hz]', 'ganancia [dB]',
               'Bode del filtro del radar: deja pasar 0,1–43 Hz', xlog=True, xdiv=7, ydiv=5)
    frames = sweep(sc, [(f, g, '')], 90, hlines=[(gmax - 3, f'−3 dB ({fmt(gmax - 3)} dB)')],
                   note='respiración y movimiento: 0,1–43 Hz', items=[('ganancia', TRACE[0])])
    save_gif('a4_radar_bode.gif', frames)


def a5_montecarlo():
    cols = read_dat(SP / 'stress_mc.dat', 6)
    vals = cols[1]  # tensión en el ADC del divisor de batería con 8,4 V
    lo, hi, bins = 2.20, 2.50, 30
    width = (hi - lo) / bins
    frames = []
    steps = list(range(5, len(vals) + 1, 5))
    final = [0] * bins
    for v in vals:
        final[min(bins - 1, max(0, int((v - lo) / width)))] += 1
    ytop = 20 * math.ceil(max(final) * 1.1 / 20)
    for n in steps:
        counts = [0] * bins
        for v in vals[:n]:
            k = min(bins - 1, max(0, int((v - lo) / width)))
            counts[k] += 1
        sc = Scope((lo, hi), (0, ytop), 'tensión en el ADC con batería llena [V]', 'corridas',
                   'Monte Carlo: 300 lotes de componentes sorteados', xdiv=6, ydiv=ytop // 20)
        note = (f'300 de 300 bajo el límite · máx. {max(vals):.3f} V'.replace('.', ',')
                if n == len(vals) else f'corrida {n} de {len(vals)}')
        img, d = sc.frame(note)
        lx, _ = sc.px(2.45, 0)
        d.line([(lx, PAD_T), (lx, H - PAD_B)], fill=LIMIT, width=3)
        d.text((lx - 8, PAD_T + 8), 'límite del ADC 2,45 V', font=F_TICK, fill=LIMIT, anchor='ra')
        for k, c in enumerate(counts):
            if not c:
                continue
            xa, ya = sc.px(lo + k * width, c)
            xb, yb = sc.px(lo + (k + 1) * width, 0)
            d.rectangle([xa + 1, ya, xb - 1, yb], fill=TRACE[0])
        frames.append(img)
    save_gif('a5_montecarlo.gif', frames, hold=FPS * 3)


def main():
    print('Animaciones del video (datos de spice/results):')
    a1_buck()
    a2_ripple()
    a3_motor()
    a4_bode()
    a5_montecarlo()


if __name__ == '__main__':
    main()
