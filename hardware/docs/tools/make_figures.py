#!/usr/bin/env python3
"""Genera las figuras del reporte de simulación (hardware/docs/figuras/*.png) a partir de los
datos REALES de las simulaciones (spice/results/*.dat, firmware/results/control/*.csv, VCD de
Wokwi) y de los diagramas en graphviz. Sin matplotlib: trazador SVG propio + Inkscape → PNG.
Uso: python3 hardware/docs/tools/make_figures.py
"""
import csv
import math
import re
import shutil
import subprocess
from pathlib import Path

HW = Path(__file__).resolve().parents[2]
SP = HW / 'spice' / 'results'
FIG = HW / 'docs' / 'figuras'
COLORS = ['#1f5fa8', '#c0392b', '#2e8b57', '#8e44ad', '#d35400']


# ------------------------------------------------------------------ utilidades de datos
def read_dat(path, ncols):
    """wrdata de ngspice: pares (escala, valor) por vector → listas [t, v1, v2, ...]."""
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


def decimate(xs, ys, n=2500):
    """Mínimo y máximo por tramo: conserva la envolvente (rizado) al reducir puntos."""
    if len(xs) <= n:
        return xs, ys
    step = len(xs) / (n / 2)
    ox, oy, k = [], [], 0.0
    while int(k) < len(xs):
        a, b = int(k), min(len(xs), int(k + step))
        seg = range(a, b)
        imin = min(seg, key=lambda i: ys[i])
        imax = max(seg, key=lambda i: ys[i])
        for i in sorted({imin, imax}):
            ox.append(xs[i])
            oy.append(ys[i])
        k += step
    return ox, oy


def nice_ticks(lo, hi, n=6):
    if hi <= lo:
        hi = lo + 1
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=10 * mag)
    start = math.ceil(lo / step) * step
    ticks, v = [], start
    while v <= hi + 1e-12:
        ticks.append(round(v, 12))
        v += step
    return ticks


def fmt(v):
    s = f'{v:.6g}'
    return s.replace('.', ',').replace('-', '−')


# ------------------------------------------------------------------ trazador SVG
def line_plot(name, series, xlabel, ylabel, xlim=None, ylim=None, hlines=(), title='', w=960, h=500,
              step=False, legend_pos='tr'):
    ml, mr, mt, mb = 82, 24, 34 if title else 18, 62
    pw, ph = w - ml - mr, h - mt - mb
    xs_all = [x for s in series for x in s[0]]
    ys_all = [y for s in series for y in s[1]] + [y for y, _ in hlines]
    x0, x1 = xlim or (min(xs_all), max(xs_all))
    y0, y1 = ylim or (min(ys_all), max(ys_all))
    if not ylim:
        pad = 0.06 * (y1 - y0 or 1)
        y0, y1 = y0 - pad, y1 + pad
    X = lambda x: ml + (x - x0) / (x1 - x0) * pw
    Y = lambda y: mt + ph - (y - y0) / (y1 - y0) * ph
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
           f'font-family="DejaVu Sans, Arial, sans-serif" font-size="14">',
           f'<rect width="{w}" height="{h}" fill="#ffffff"/>']
    if title:
        out.append(f'<text x="{ml}" y="22" font-size="15" font-weight="bold" fill="#222">{title}</text>')
    for t in nice_ticks(x0, x1):
        out.append(f'<line x1="{X(t):.1f}" y1="{mt}" x2="{X(t):.1f}" y2="{mt + ph}" stroke="#e3e3e3"/>')
        out.append(f'<text x="{X(t):.1f}" y="{mt + ph + 20}" text-anchor="middle" fill="#333">{fmt(t)}</text>')
    for t in nice_ticks(y0, y1):
        out.append(f'<line x1="{ml}" y1="{Y(t):.1f}" x2="{ml + pw}" y2="{Y(t):.1f}" stroke="#e3e3e3"/>')
        out.append(f'<text x="{ml - 8}" y="{Y(t) + 5:.1f}" text-anchor="end" fill="#333">{fmt(t)}</text>')
    out.append(f'<rect x="{ml}" y="{mt}" width="{pw}" height="{ph}" fill="none" stroke="#555"/>')
    out.append(f'<text x="{ml + pw / 2}" y="{h - 14}" text-anchor="middle" fill="#222">{xlabel}</text>')
    out.append(f'<text transform="translate(20,{mt + ph / 2}) rotate(-90)" text-anchor="middle" fill="#222">{ylabel}</text>')
    out.append(f'<clipPath id="c"><rect x="{ml}" y="{mt}" width="{pw}" height="{ph}"/></clipPath><g clip-path="url(#c)">')
    for y, lab in hlines:
        out.append(f'<line x1="{ml}" y1="{Y(y):.1f}" x2="{ml + pw}" y2="{Y(y):.1f}" stroke="#777" stroke-dasharray="6 4"/>')
        out.append(f'<text x="{ml + pw - 6}" y="{Y(y) - 5:.1f}" text-anchor="end" fill="#555" font-size="12">{lab}</text>')
    for k, (xs, ys, lab) in enumerate(series):
        xs2, ys2 = decimate(xs, ys)
        pts = []
        for i, (x, y) in enumerate(zip(xs2, ys2)):
            if step and i:
                pts.append(f'{X(x):.2f},{Y(ys2[i - 1]):.2f}')
            pts.append(f'{X(x):.2f},{Y(y):.2f}')
        out.append(f'<polyline fill="none" stroke="{COLORS[k % 5]}" stroke-width="1.6" points="{" ".join(pts)}"/>')
    out.append('</g>')
    lx = ml + pw - 250 if legend_pos == 'tr' else ml + 12
    ly = mt + 10 if legend_pos in ('tr', 'tl') else mt + ph - 22 * len(series) - 6
    for k, (_, _, lab) in enumerate(series):
        yy = ly + 22 * k
        out.append(f'<rect x="{lx - 6}" y="{yy - 4}" width="250" height="22" fill="#ffffff" fill-opacity="0.85"/>')
        out.append(f'<line x1="{lx}" y1="{yy + 7}" x2="{lx + 26}" y2="{yy + 7}" stroke="{COLORS[k % 5]}" stroke-width="3"/>')
        out.append(f'<text x="{lx + 34}" y="{yy + 12}" fill="#222" font-size="13">{lab}</text>')
    out.append('</svg>')
    save(name, '\n'.join(out))


def histogram(name, values, xlabel, limits=(), bins=24, w=620, h=380, title=''):
    lo, hi = min(values), max(values)
    for v, _ in limits:
        lo, hi = min(lo, v), max(hi, v)
    pad = 0.05 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    width = (max(values) - min(values)) / bins or 1
    edges = [min(values) + i * width for i in range(bins + 1)]
    counts = [0] * bins
    for v in values:
        counts[min(bins - 1, int((v - min(values)) / width))] += 1
    ml, mr, mt, mb = 64, 16, 34 if title else 16, 58
    pw, ph = w - ml - mr, h - mt - mb
    ymax = max(counts) * 1.1
    X = lambda x: ml + (x - lo) / (hi - lo) * pw
    Y = lambda c: mt + ph - c / ymax * ph
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
           f'font-family="DejaVu Sans, Arial, sans-serif" font-size="13"><rect width="{w}" height="{h}" fill="#fff"/>']
    if title:
        out.append(f'<text x="{ml}" y="22" font-size="14" font-weight="bold" fill="#222">{title}</text>')
    for t in nice_ticks(lo, hi, 5):
        out.append(f'<text x="{X(t):.1f}" y="{mt + ph + 18}" text-anchor="middle" fill="#333">{fmt(t)}</text>')
    for t in nice_ticks(0, ymax, 4):
        out.append(f'<line x1="{ml}" y1="{Y(t):.1f}" x2="{ml + pw}" y2="{Y(t):.1f}" stroke="#e6e6e6"/>')
        out.append(f'<text x="{ml - 6}" y="{Y(t) + 4:.1f}" text-anchor="end" fill="#333">{fmt(t)}</text>')
    for c, a, b in zip(counts, edges, edges[1:]):
        out.append(f'<rect x="{X(a):.1f}" y="{Y(c):.1f}" width="{max(1, X(b) - X(a) - 1):.1f}" height="{mt + ph - Y(c):.1f}" fill="{COLORS[0]}"/>')
    for v, lab in limits:
        out.append(f'<line x1="{X(v):.1f}" y1="{mt}" x2="{X(v):.1f}" y2="{mt + ph}" stroke="{COLORS[1]}" stroke-dasharray="6 4" stroke-width="2"/>')
        out.append(f'<text x="{X(v) + 4:.1f}" y="{mt + 14}" fill="{COLORS[1]}" font-size="12">{lab}</text>')
    out.append(f'<rect x="{ml}" y="{mt}" width="{pw}" height="{ph}" fill="none" stroke="#555"/>')
    out.append(f'<text x="{ml + pw / 2}" y="{h - 12}" text-anchor="middle" fill="#222">{xlabel}</text>')
    out.append(f'<text transform="translate(16,{mt + ph / 2}) rotate(-90)" text-anchor="middle" fill="#222">corridas</text></svg>')
    save(name, '\n'.join(out))


def save(name, svg):
    p = FIG / f'{name}.svg'
    p.write_text(svg, encoding='utf-8')
    to_png(p)


def to_png(svg_path, width=1400):
    png = svg_path.with_suffix('.png')
    subprocess.run(['inkscape', str(svg_path), '--export-type=png', f'--export-filename={png}',
                    f'--export-width={width}', '--export-background=#ffffff', '--export-background-opacity=1'],
                   check=True, capture_output=True)
    svg_path.unlink()


def dot(name, source):
    p = FIG / f'{name}.dot'
    p.write_text(source, encoding='utf-8')
    subprocess.run(['dot', '-Tpng', '-Gdpi=150', str(p), '-o', str(FIG / f'{name}.png')], check=True)
    p.unlink()


# ------------------------------------------------------------------ figuras
def fig_power():
    for tag, vb in (('74', '7,4'), ('62', '6,2')):
        t, vout, vbat, il = read_dat(SP / f'power_supply_{tag}.dat', 3)
        tm = [x * 1e3 for x in t]
        line_plot(f'r1_alimentacion_{tag}', [(tm, vout, '5V_USB (salida del regulador)'), (tm, vbat, 'VBAT (después del IRF4905)')],
                  'tiempo [ms]', 'tensión [V]', xlim=(0.5, 3.2), ylim=(4.4, 8.0),
                  hlines=[(4.75, 'mínimo USB 4,75 V'), (5.5, 'margen del regulador 5,5 V')],
                  title=f'Alimentación con batería de {vb} V: arranque de motores (1 ms) y carga USB máxima (2 ms)')
    t, vout, vbat, il = window(read_dat(SP / 'power_supply_74.dat', 3), 0.80e-3, 0.84e-3)
    line_plot('r1_rizado_74', [([x * 1e6 for x in t], vout, '5V_USB')], 'tiempo [µs]', 'tensión [V]',
              title='Rizado de 5V_USB en régimen (batería de 7,4 V), osciloscopio virtual')


def fig_motor():
    t, i, w, vm = read_dat(SP / 'motor_start.dat', 3)
    line_plot('r1_motor_arranque', [(t, i, 'corriente del motor [A]'), (t, [x * 0.0325 for x in w], 'velocidad de la rueda [m/s]')],
              'tiempo [s]', 'corriente [A] / velocidad [m/s]', xlim=(0, 0.4),
              hlines=[(1.2, 'corriente media TB6612 1,2 A')], title='Arranque a duty máximo (5,0 V efectivos, batería llena)')
    t2, i2, vm2 = window([t, i, vm], 0.399, 0.4)
    line_plot('r1_motor_pwm', [([x * 1e3 for x in t2], i2, 'corriente del motor [A]')], 'tiempo [ms]', 'corriente [A]',
              title='Corriente del motor en régimen: rizado del PWM de 20 kHz')
    ts, ist = read_dat(SP / 'motor_stall.dat', 1)
    line_plot('r1_motor_bloqueado', [([x * 1e3 for x in ts], ist, 'rueda bloqueada, 5,0 V efectivos')], 'tiempo [ms]', 'corriente [A]',
              hlines=[(1.2, 'corriente media TB6612 1,2 A')], ylim=(0, 1.5), title='Rueda bloqueada (peor caso de carga)')


def fig_i2c():
    t, a, b, c = read_dat(SP / 'i2c_bus.dat', 3)
    line_plot('r1_i2c', [([x * 1e6 for x in t], a, 'A: 2,2 kΩ ∥ 4,7 kΩ (GY-521), 100 pF'),
                         ([x * 1e6 for x in t], b, 'B: solo 2,2 kΩ, 100 pF'), ([x * 1e6 for x in t], c, 'C: 2,2 kΩ ∥ 4,7 kΩ, 200 pF')],
              'tiempo [µs]', 'SDA [V]', xlim=(2.4, 5.4), ylim=(-0.2, 3.6),
              hlines=[(2.31, '70 % de VDD'), (0.99, '30 % de VDD')], title='Bus I2C a 400 kHz: subida de SDA', legend_pos='bl')


def fig_mc():
    cols = read_dat(SP / 'stress_mc.dat', 6)
    histogram('r3_mc_bateria', cols[1], 'tensión en el ADC con 8,4 V [V]', limits=[(2.45, 'máx. 2,45 V')], title='Divisor de batería (300 corridas)')
    histogram('r3_mc_co2', cols[2], 'ganancia del amplificador del CO₂', limits=[(2.94, '−2 %'), (3.06, '+2 %')], title='Amplificador del CO₂')
    histogram('r3_mc_radar_fh', cols[6], 'corte superior del radar [Hz]', limits=[(35, 'mín. 35 Hz')], title='Radar: corte superior')
    histogram('r3_mc_radar_fl', cols[5], 'corte inferior del radar [Hz]', limits=[(0.15, 'máx. 0,15 Hz')], title='Radar: corte inferior')


def fig_noise():
    t, n, s = read_dat(SP / 'stress_noise.dat', 2)
    line_plot('r3_ruido_radar', [(t, s, 'señal + ruido'), (t, n, 'solo ruido')], 'tiempo [s]', 'salida del radar [V]',
              xlim=(4, 12), ylim=(0, 2.6), hlines=[(2.45, 'límite fiable del ADC 2,45 V'), (0.15, '0,15 V')],
              title='Radar con ruido blanco en la entrada (10 µV rms supuesto)')


def fig_control():
    d = HW / 'firmware' / 'results' / 'control'
    series = []
    for name, lab in (('nominal', 'con PID, nominal'), ('carga_x3', 'con PID, carga ×3'),
                      ('ruido_encoder', 'con PID, ruido de encoder'), ('sin_pid', 'sin PID (referencia)')):
        rows = list(csv.DictReader(open(d / f'{name}.csv')))
        series.append(([float(r['t']) for r in rows], [float(r['v_odom']) for r in rows], lab))
    rows = list(csv.DictReader(open(d / 'nominal.csv')))
    series.insert(0, ([float(r['t']) for r in rows], [float(r['v_ref']) for r in rows], 'consigna limitada'))
    line_plot('r2_pid', series, 'tiempo [s]', 'velocidad del robot (odometría) [m/s]', xlim=(0, 4), ylim=(-0.02, 0.36),
              title='Emulación del lazo de control con el código del firmware', step=True, legend_pos='tl')


def fig_vcd():
    """Traza del analizador lógico (Wokwi, prueba micro-ROS) — evidencia real."""
    vcd = (HW / 'docs' / 'evidencia' / 'wokwi_microros_e2e_2026-11-20_1648.vcd').read_text()
    ids = dict(re.findall(r'\$var wire 1 (\S+) (D\d) \$end', vcd))
    names = {'D0': 'MOT_L_PWM', 'D1': 'MOT_L_IN1', 'D2': 'MOT_L_IN2', 'D3': 'MOT_R_PWM', 'D4': 'MOT_R_IN1',
             'D5': 'MOT_R_IN2', 'D6': 'MOT_STBY', 'D7': 'CO2_HEATER'}
    ev = {k: [] for k in ids}
    t = 0
    for line in vcd.splitlines():
        if line.startswith('#'):
            t = int(line[1:]) * 1e-9
        elif line[:1] in '01' and line[1:] in ids:
            ev[line[1:]].append((t, int(line[0])))
    tend = max(t, 1.0)
    w, h, ml = 960, 60 + 42 * 8, 130
    X = lambda x: ml + x / tend * (w - ml - 20)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" font-family="DejaVu Sans, Arial" font-size="13">'
           f'<rect width="{w}" height="{h}" fill="#fff"/>']
    for k, (sym, ch) in enumerate(sorted(ids.items(), key=lambda kv: kv[1])):
        y0 = 20 + 42 * k
        pts, lvl = [], 0
        for tt, v in ev[sym]:
            pts += [f'{X(tt):.1f},{y0 + 28 - 22 * lvl}', f'{X(tt):.1f},{y0 + 28 - 22 * v}']
            lvl = v
        pts.append(f'{X(tend):.1f},{y0 + 28 - 22 * lvl}')
        out.append(f'<text x="8" y="{y0 + 22}" fill="#222">{ch} {names[ch]}</text>')
        out.append(f'<polyline fill="none" stroke="{COLORS[k % 5]}" stroke-width="2" points="{" ".join(pts)}"/>')
    for tk in nice_ticks(0, tend, 7):
        out.append(f'<text x="{X(tk):.1f}" y="{h - 14}" text-anchor="middle" fill="#333">{fmt(tk)} s</text>')
    out.append('</svg>')
    save('r2_analizador_logico', '\n'.join(out))


def fig_h2_svgs():
    for f in sorted(SP.glob('*.svg')):
        tmp = FIG / f'h2_{f.name}'
        shutil.copy(f, tmp)
        to_png(tmp)


def fig_diagrams():
    dot('r4_bloques', r'''digraph G {
  rankdir=LR; splines=true; nodesep=0.35; ranksep=0.9;
  node [shape=box, style="rounded,filled", fontname="DejaVu Sans", fontsize=11, fillcolor="#eef3f8", color="#3b5b7a"];
  edge [fontname="DejaVu Sans", fontsize=9, color="#444444"];
  subgraph cluster_e { label="Energía"; style=dashed; color="#999999";
    BAT [label="Batería 2S Li-ion\n7,4 V · 3 Ah + BMS"]; PROT [label="Interruptor + fusible 5 A\n+ IRF4905 (inversión)"];
    B1 [label="MP1584 → 5V_USB\n3 A"]; B2 [label="MP1584 + LC → 5V_AUX"]; }
  subgraph cluster_o { label="Sensores optoelectrónicos y USB"; style=dashed; color="#999999";
    LIDAR [label="RPLiDAR A1\nláser, triangulación\n0,15–12 m · 5,5 Hz", fillcolor="#fdf2e3", color="#b9770e"];
    CAM [label="RealSense D435\nRGB + estéreo IR\n+ proyector IR", fillcolor="#fdf2e3", color="#b9770e"];
    MIC [label="ReSpeaker\n4 micrófonos"]; HUB [label="Hub USB 3.0\nalimentado"]; }
  subgraph cluster_m { label="Placa base"; style=dashed; color="#999999";
    ESP [label="ESP32 DevKit V1\ncontrol + micro-ROS", fillcolor="#e6f4ea", color="#2e7d32"];
    DRV [label="TB6612FNG\npuente H doble"]; MOT [label="2 motores TT 1:48\n+ encoders Hall"];
    IMU [label="MPU6050\nIMU 6 ejes"]; THERM [label="MLX90640\ntermopila IR 32×24\n(fase 2)", fillcolor="#fdf2e3", color="#b9770e"];
    ANA [label="MCP6004\nacondicionamiento"]; CO2 [label="TGS4161\nCO₂"]; RAD [label="HB100\nradar 10,525 GHz"]; }
  PC [label="Portátil\nROS 2 Jazzy\nSLAM · Nav2 · detección", fillcolor="#e6f4ea", color="#2e7d32"];
  BAT -> PROT [label="VBAT"]; PROT -> B1; PROT -> B2; PROT -> DRV [label="VM"];
  B1 -> HUB [label="5 V"]; B2 -> CO2 [label="calefactor"]; B2 -> RAD [label="5 V"];
  HUB -> PC [label="USB 3.0 activo ~5 m", dir=both];
  LIDAR -> HUB [label="USB"]; CAM -> HUB [label="USB 3.0"]; MIC -> HUB [label="USB"]; ESP -> HUB [label="USB serial\n921 600 Bd", dir=both];
  ESP -> DRV [label="PWM 20 kHz\n+ dirección"]; DRV -> MOT [label="AO/BO"]; MOT -> ESP [label="cuadratura\nPCNT ×4"];
  IMU -> ESP [label="I2C 400 kHz", dir=both]; THERM -> ESP [label="I2C", dir=both];
  CO2 -> ANA [label="FEM"]; RAD -> ANA [label="IF"]; ANA -> ESP [label="ADC1"]; PROT -> ESP [label="divisor\nADC1"];
}''')
    dot('r2_estados_traccion', r'''digraph G {
  rankdir=LR; node [shape=ellipse, fontname="DejaVu Sans", fontsize=11, style=filled, fillcolor="#eef3f8"];
  edge [fontname="DejaVu Sans", fontsize=9];
  start [shape=point]; IDLE [label="Reposo\n(kIdle)"]; DRIVING [label="En marcha\n(kDriving)", fillcolor="#e6f4ea"];
  STALE [label="Sin mando\n(kStale)", fillcolor="#fdf2e3"]; CRIT [label="Batería crítica\n(kCritical)\nSTBY = 0", fillcolor="#fbe3e1"];
  start -> IDLE; IDLE -> DRIVING [label="mando ≠ 0"]; DRIVING -> IDLE [label="mando = 0"];
  DRIVING -> STALE [label="0,5 s sin /cmd_vel"]; STALE -> DRIVING [label="nuevo mando ≠ 0"];
  IDLE -> CRIT [label="VBAT < 6,2 V"]; DRIVING -> CRIT [label="VBAT < 6,2 V"]; STALE -> CRIT [label="VBAT < 6,2 V"];
  CRIT -> IDLE [label="VBAT > 6,7 V\n(histéresis)"];
}''')
    dot('r2_estados_agente', r'''digraph G {
  rankdir=LR; node [shape=ellipse, fontname="DejaVu Sans", fontsize=11, style=filled, fillcolor="#eef3f8"];
  edge [fontname="DejaVu Sans", fontsize=9];
  start [shape=point]; W [label="Esperando agente"]; A [label="Agente disponible"];
  C [label="Conectado\npublica y escucha", fillcolor="#e6f4ea"]; D [label="Desconectado", fillcolor="#fbe3e1"];
  start -> W; W -> W [label="ping cada 0,5 s sin respuesta"]; W -> A [label="ping OK"];
  A -> C [label="entidades creadas"]; A -> W [label="falla al crear"]; C -> C [label="ping OK cada 1 s"];
  C -> D [label="3 pings fallidos"]; D -> W [label="destruir entidades"];
}''')
    dot('r2_flujo_control', r'''digraph G {
  node [fontname="DejaVu Sans", fontsize=10, shape=box, style="rounded,filled", fillcolor="#eef3f8"];
  edge [fontname="DejaVu Sans", fontsize=9];
  s [label="Cada 20 ms (50 Hz)", shape=oval, fillcolor="#e6f4ea"];
  enc [label="Leer cuentas del PCNT\n→ odometría (x, y, θ, v, ω)"];
  crit [label="¿Batería crítica?", shape=diamond, fillcolor="#fdf2e3"];
  off [label="STBY = 0, reiniciar PID\n(motores apagados)", fillcolor="#fbe3e1"];
  stale [label="¿0,5 s sin mando?", shape=diamond, fillcolor="#fdf2e3"];
  zero [label="consigna = 0"]; cmd [label="consigna = último /cmd_vel"];
  lim [label="Limitador: v ≤ 0,30 m/s, ω ≤ 1,5 rad/s\na ≤ 0,5 m/s²"];
  ik [label="Cinemática inversa → ω izquierda, ω derecha"];
  pid [label="u = ω_ref / ω_máx + PID(ω_ref, ω_medida)"];
  duty [label="duty = zona muerta + (V_lím/VBAT − zm)·|u|\n(V_lím = 5,0 V)"];
  pwm [label="PWM 20 kHz + IN1/IN2 al TB6612"];
  s -> enc -> crit; crit -> off [label="sí"]; crit -> stale [label="no"];
  stale -> zero [label="sí"]; stale -> cmd [label="no"]; zero -> lim; cmd -> lim; lim -> ik -> pid -> duty -> pwm;
}''')


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    for f in (fig_power, fig_motor, fig_i2c, fig_mc, fig_noise, fig_control, fig_vcd, fig_h2_svgs, fig_diagrams):
        f()
        print('ok', f.__name__)
    print(len(list(FIG.glob('*.png'))), 'figuras en', FIG)


if __name__ == '__main__':
    main()
