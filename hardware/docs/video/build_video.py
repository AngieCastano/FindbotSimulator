#!/usr/bin/env python3
"""Monta el video de la simulación del hardware a partir del guion GUION_VIDEO_HARDWARE.md.

Todo sale de archivos reales del proyecto:
  - figuras/                   diagrama de bloques, esquemáticos de KiCad, máquina de estados,
                               analizador lógico de Wokwi
  - spice/results/*.dat        curvas de ngspice (se vuelven a graficar en estilo osciloscopio)
  - spice/results/summary.txt  valores de los criterios (rizado, cortes del filtro, corrientes...)
  - spice/radar_amp.cir        se vuelve a correr el análisis AC para el diagrama de Bode
  - kicad/out/erc.rpt          resultado del ERC
  - docs/evidencia/*.log       log serial de Wokwi (escenario H5) y su telemetría de batería
  - docs/REPORTE_SIMULACION.md tablas 13 (pasa/no pasa) y 14 (correcciones)
  - docs/GUION_VIDEO_HARDWARE.md narración (subtítulos) y tiempos de cada escena

Salida en hardware/docs/video/:
  imagenes/*.png                         una imagen 1920×1080 por toma
  subtitulos.srt                         narración en subtítulos (máx. 2 líneas de ~42 caracteres)
  video_hardware_sin_subtitulos.mp4
  video_hardware_con_subtitulos.mp4      subtítulos quemados (blanco con borde negro)
  preview/*.png                          6 fotogramas de la versión con subtítulos

Si existe hardware/docs/video/narracion.mp3 se usa como audio y todos los tiempos (imágenes y
subtítulos) se escalan a su duración (máx. 3:00). Si no existe, el video sale sin voz (pista
de audio en silencio).

Uso:  python hardware/docs/video/build_video.py            (todo)
      python hardware/docs/video/build_video.py --solo-imagenes  (solo imágenes y .srt)
Requiere: Python 3 + matplotlib + numpy + PyYAML + Pillow, ffmpeg (libx264, libass), ngspice.
"""
import json
import re
import shutil
import sys
import subprocess
import tempfile
import textwrap
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402
from PIL import Image  # noqa: E402

VID = Path(__file__).resolve().parent
HW = VID.parents[1]
DOCS = HW / 'docs'
FIG = DOCS / 'figuras'
SP = HW / 'spice' / 'results'
IMG = VID / 'imagenes'
PREV = VID / 'preview'
BUILD = VID / 'build'          # clips intermedios (ignorado por git)
AUDIO = VID / 'narracion.mp3'
SRT = VID / 'subtitulos.srt'
OUT_SIN = VID / 'video_hardware_sin_subtitulos.mp4'
OUT_CON = VID / 'video_hardware_con_subtitulos.mp4'

W, H, FPS = 1920, 1080, 30
FADE = 0.5                     # s, fundido entre imágenes
ZOOM = 0.05                    # Ken Burns: 5 % de zoom a lo largo de cada imagen
MAX_S = 180.0                  # duración máxima del video

# paleta
BG = '#0e1116'
PANEL = '#161b22'
FG = '#e6edf3'
MUTED = '#8b949e'
ACCENT = '#3fb950'             # verde "pasa"
WARN = '#f0b429'
LIMIT = '#ff6b6b'
C1, C2, C3 = '#3fb950', '#f0b429', '#58a6ff'

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 20, 'axes.facecolor': PANEL,
    'axes.edgecolor': '#30363d', 'axes.labelcolor': FG, 'xtick.color': FG, 'ytick.color': FG,
    'grid.color': '#30363d', 'axes.grid': True, 'figure.facecolor': BG, 'text.color': FG,
    'axes.titlecolor': FG, 'legend.facecolor': PANEL, 'legend.edgecolor': '#30363d',
    'axes.formatter.use_locale': False,
})

CONTENT = [0.06, 0.26, 0.88, 0.60]   # zona del contenido: deja la franja inferior a los subtítulos


# ============================================================ utilidades
def comma(x, nd=2):
    return f'{x:.{nd}f}'.replace('.', ',')


def summary():
    """summary.txt de check_spice → {(circuito, medida): valor} y total de criterios/fallas."""
    vals = {}
    text = (SP / 'summary.txt').read_text()
    for line in text.splitlines():
        p = line.split()
        if len(p) >= 4 and p[0] in ('OK', 'FALLA', 'FAIL'):
            vals[(p[1], p[2])] = float(p[3])
    m = re.search(r'(\d+) criterios, (\d+) fallas', text)
    return vals, int(m.group(1)), int(m.group(2)), text


def read_dat(path, ncols):
    """wrdata de ngspice: pares (escala, valor) → arreglo [t, v1, v2, ...]."""
    a = np.loadtxt(path, usecols=[0] + [2 * k + 1 for k in range(ncols)])
    return a.T


def slide(label, tag=''):
    """Lienzo 1920×1080 con el rótulo (texto en pantalla) arriba y la etiqueta de la escena."""
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    if tag:
        fig.text(0.06, 0.968, tag, fontsize=20, color=MUTED, va='center')
    if label:
        fig.text(0.06, 0.885, label, fontsize=40, weight='bold', color=FG, va='center',
                 bbox=dict(boxstyle='round,pad=0.35', fc='#1f6f3f' if '✔' in label else '#21262d',
                           ec='none'))
    return fig


def save(fig, name):
    path = IMG / f'{name}.png'
    fig.savefig(path, dpi=100, facecolor=BG)
    plt.close(fig)
    return path


def show_image(fig, path, rect=CONTENT, crop=None, boxes=()):
    """Imagen real del proyecto, centrada en rect, con recuadros opcionales (px de la imagen)."""
    im = Image.open(path).convert('RGB')
    if crop:
        im = im.crop(crop)
    ax = fig.add_axes(rect)
    ax.imshow(np.asarray(im), interpolation='lanczos')
    ax.set_axis_off()
    for (x0, y0, x1, y1, color) in boxes:
        ox, oy = (crop[0], crop[1]) if crop else (0, 0)
        ax.add_patch(Rectangle((x0 - ox, y0 - oy), x1 - x0, y1 - y0, fill=False, ec=color, lw=5))
    return ax


def note(fig, text, x=0.06, y=0.205, color=MUTED, size=19):
    fig.text(x, y, text, fontsize=size, color=color, va='top')


def callout(ax, text, xy, xytext, color=WARN):
    ax.annotate(text, xy=xy, xytext=xytext, color=color, fontsize=24, weight='bold',
                arrowprops=dict(arrowstyle='->', color=color, lw=3),
                bbox=dict(boxstyle='round,pad=0.3', fc=BG, ec=color, lw=2))


def terminal(fig, lines, rect=CONTENT, highlight=(), size=22, title=''):
    """Texto real (log, informe) en una ventana tipo terminal; resalta las líneas indicadas."""
    ax = fig.add_axes(rect)
    ax.set_axis_off()
    ax.add_patch(FancyBboxPatch((0, 0), 1, 1, boxstyle='round,pad=0,rounding_size=0.015',
                                transform=ax.transAxes, fc='#010409', ec='#30363d', lw=2))
    if title:
        ax.text(0.015, 0.965, title, transform=ax.transAxes, fontsize=16, color=MUTED, va='top',
                family='DejaVu Sans Mono')
    line_px = size * 100 / 72 * 1.55                 # alto de línea en píxeles
    step = line_px / (rect[3] * H)                   # en fracción del alto de la ventana
    top = 1 - (48 if title else 16) / (rect[3] * H)
    lines = [l if len(l) <= 92 else l[:91] + '…' for l in lines]
    for i, line in enumerate(lines):
        y = top - i * step
        hl = i in highlight
        if hl:
            ax.add_patch(Rectangle((0.008, y - step * 0.78), 0.984, step * 0.9,
                                   transform=ax.transAxes, fc='#1f6f3f', ec=ACCENT, lw=2))
        ax.text(0.02, y, line, transform=ax.transAxes, fontsize=size, va='top',
                family='DejaVu Sans Mono', color='#ffffff' if hl else '#c9d1d9',
                weight='bold' if hl else 'normal')
    return ax


def scope_axes(fig, rect=CONTENT):
    ax = fig.add_axes([rect[0] + 0.04, rect[1] + 0.03, rect[2] - 0.05, rect[3] - 0.05])
    ax.set_facecolor('#06090d')
    ax.grid(True, color='#1d3b2a', lw=1)
    for s in ax.spines.values():
        s.set_color('#2d4f3a')
    fmt = FuncFormatter(lambda v, p: f'{v:.6g}'.replace('.', ','))
    ax.xaxis.set_major_formatter(fmt)
    ax.yaxis.set_major_formatter(fmt)
    return ax


def md_table(title):
    """Primera tabla markdown después de '**Tabla N**' en REPORTE_SIMULACION.md."""
    text = (DOCS / 'REPORTE_SIMULACION.md').read_text()
    seg = text.split(f'**{title}**', 1)[1]
    rows = [l for l in seg.splitlines() if l.startswith('|')]
    out = []
    for l in rows:
        if set(l.replace('|', '').strip()) <= {'-', ' '}:
            if out:
                continue
        cells = [c.strip().replace('`', '') for c in l.strip('|').split('|')]
        out.append(cells)
        if len(out) > 1 and not seg.split(l, 1)[1].lstrip('\n').startswith('|'):
            break
    return out[0], out[1:]


def draw_table(fig, header, rows, widths, rect=CONTENT, size=17, wrap=None, pass_col=None):
    ax = fig.add_axes(rect)
    ax.set_axis_off()
    wrap = wrap or [200] * len(header)
    cells = [['\n'.join(textwrap.wrap(c, w)) for c, w in zip(r, wrap)] for r in rows]
    t = ax.table(cellText=cells, colLabels=header, colWidths=widths, loc='upper center',
                 cellLoc='left')
    t.auto_set_font_size(False)
    t.set_fontsize(size)
    nlines = [max(c.count('\n') + 1 for c in r) for r in cells]
    unit = 1.0 / (sum(nlines) + 1.3)
    for (r, c), cell in t.get_celld().items():
        cell.set_edgecolor('#30363d')
        if r == 0:
            cell.set_facecolor('#21262d')
            cell.get_text().set_weight('bold')
            cell.set_height(unit * 1.3)
        else:
            cell.set_facecolor(PANEL if r % 2 else '#1b2129')
            cell.set_height(unit * nlines[r - 1])
            if pass_col is not None and c == pass_col:
                ok = cell.get_text().get_text().strip().lower().startswith('s')
                cell.get_text().set_text('✔ Sí' if ok else '✘ No')
                cell.get_text().set_color(ACCENT if ok else LIMIT)
                cell.get_text().set_weight('bold')
        cell.get_text().set_color(cell.get_text().get_color() if (pass_col is not None and c == pass_col and r)
                                  else FG)
    return ax


def serial_log():
    path = DOCS / 'evidencia' / 'wokwi_h5_serial_2026-11-20_1644.log'
    raw = path.read_text(errors='replace').splitlines()
    # Los avisos "# ..." comparten línea con la telemetría "T,..." (el monitor no separa): se
    # muestran por separado.
    msgs = [re.sub(r'T,\d+,.*$', '', l).rstrip() for l in raw]
    header = next(l for l in raw if l.startswith('# T,')).lstrip('# ').split(',')
    tele = []
    for l in raw:
        m = re.search(r'(T,\d+,.*)$', l)
        if m and not l.startswith('# T,'):
            p = m.group(1).split(',')
            if len(p) == len(header):
                tele.append(p)
    return path, raw, msgs, header, tele


# ============================================================ imágenes por toma
def img_title():
    fig = slide('')
    fig.text(0.5, 0.62, 'findbot', fontsize=110, weight='bold', ha='center', color=FG)
    fig.text(0.5, 0.50, 'Validación virtual del circuito', fontsize=46, ha='center', color=ACCENT)
    fig.text(0.5, 0.40, 'Robot autónomo de búsqueda de víctimas entre escombros', fontsize=28,
             ha='center', color=MUTED)
    fig.text(0.5, 0.33, 'Proyecto de grado · Ingeniería Electrónica · UNAD', fontsize=24,
             ha='center', color=MUTED)
    return save(fig, '00_titulo')


def img_collage(parts):
    fig = slide('findbot · validación virtual del circuito', 'Introducción')
    rects = [[0.05, 0.545, 0.44, 0.25], [0.51, 0.545, 0.44, 0.25],
             [0.05, 0.24, 0.44, 0.28], [0.51, 0.24, 0.44, 0.28]]
    for (p, cap), r in zip(parts, rects):
        ax = show_image(fig, p, r)
        ax.set_title(cap, fontsize=18, color=MUTED, pad=6)
    return save(fig, '01_collage')


R4 = FIG / 'r4_bloques.png'


def img_r4(name, label, crop, cap):
    fig = slide(label, 'Capa física (R4) · diagrama de bloques')
    show_image(fig, R4, crop=crop)
    note(fig, cap)
    return save(fig, name)


def img_sch(name, label, file, cap):
    fig = slide(label, 'Esquemático en KiCad 10')
    show_image(fig, FIG / file)
    note(fig, cap)
    return save(fig, name)


def img_erc():
    rpt = (HW / 'kicad' / 'out' / 'erc.rpt').read_text().splitlines()
    lines = [l for l in rpt if l.strip()][:12]
    hl = [i for i, l in enumerate(lines) if 'ERC messages' in l]
    m = re.search(r'(\d+)\s+Errors\s+(\d+)\s+Warnings', '\n'.join(rpt))
    errs, warns = int(m.group(1)), int(m.group(2))
    fig = slide(f'ERC: {errs} errores ✔' if errs == 0 else f'ERC: {errs} errores', 'Esquemático en KiCad 10 · revisión eléctrica (ERC)')
    terminal(fig, lines, highlight=hl, size=21, title='hardware/kicad/out/erc.rpt')
    note(fig, f'Comprobador de reglas eléctricas de KiCad: {errs} errores y {warns} advertencias.')
    return save(fig, '09_erc')


def img_summary(text, n, nf):
    keep = [l for l in text.splitlines()
            if l.startswith('OK') and re.search(r'power_supply\s+(ripple|vout_avg)|motor_drive\s+i_|radar_amp\s+(gain|flow|fhigh)', l)]
    lines = ['$ ./run_spice.sh', '...'] + keep + ['...', '', f'{n} criterios, {nf} fallas']
    fig = slide('ngspice: 11 circuitos', 'Simulación de circuitos y potencia (R1) · ngspice 42')
    terminal(fig, lines, highlight=[len(lines) - 1], size=19, title='hardware/spice/results/summary.txt (extracto)')
    note(fig, 'Cada línea: circuito, medida, valor y rango aceptado [mín, máx].')
    return save(fig, '10_spice_resumen')


def img_power(vals):
    t, vout, vbat, _ = read_dat(SP / 'power_supply_74.dat', 3)
    k = (t >= 0.5e-3) & (t <= 3.2e-3)
    fig = slide('5 V estable ✔', 'R1 · Alimentación · osciloscopio virtual (ngspice)')
    ax = scope_axes(fig)
    ax.plot(t[k] * 1e3, vbat[k], color=C2, lw=2.5, label='VBAT (batería de 7,4 V)')
    ax.plot(t[k] * 1e3, vout[k], color=C1, lw=2.5, label='5V_USB (salida del regulador)')
    ax.axhline(4.75, color=LIMIT, ls='--', lw=2)
    ax.text(3.18, 4.70, 'mínimo USB 4,75 V', color=LIMIT, ha='right', va='top', fontsize=18)
    ax.set_ylim(4.4, 8.0)
    ax.set_xlabel('tiempo [ms]')
    ax.set_ylabel('tensión [V]')
    ax.legend(loc='upper right', fontsize=18)
    dip = vals[('power_supply', 'vout_dip_74')]
    i = np.argmin(np.where(k & (t > 0.95e-3), vout, 99))
    callout(ax, f'caída mínima {comma(dip, 3)} V', (t[i] * 1e3, vout[i]), (t[i] * 1e3 + 0.25, 5.6))
    ax.annotate('arrancan los motores', xy=(1.0, 7.2), xytext=(1.15, 7.6), color=MUTED, fontsize=18,
                arrowprops=dict(arrowstyle='->', color=MUTED))
    return save(fig, '11_alimentacion')


def img_ripple(vals):
    t, vout, _, _ = read_dat(SP / 'power_supply_74.dat', 3)
    k = (t >= 0.7e-3) & (t <= 0.95e-3)      # misma ventana que mide power_supply.cir
    tt, vv = t[k] * 1e3, vout[k]
    pp = (vv.max() - vv.min()) * 1e3
    ref = vals[('power_supply', 'ripple_mv_74')]
    assert abs(pp - ref) < 0.5, f'rizado {pp} mV no coincide con summary.txt ({ref} mV)'
    worst = max(vals[('power_supply', f'ripple_mv_{x}')] for x in ('84', '74', '62'))
    fig = slide('Rizado < 50 mV ✔', 'R1 · Alimentación · zoom al rizado de 5V_USB')
    ax = scope_axes(fig)
    ax.plot(tt, vv, color=C1, lw=1.8)
    ax.axhline(vv.max(), color=WARN, ls=':', lw=2)
    ax.axhline(vv.min(), color=WARN, ls=':', lw=2)
    xm = tt[0] + 0.85 * (tt[-1] - tt[0])
    ax.annotate('', xy=(xm, vv.max()), xytext=(xm, vv.min()),
                arrowprops=dict(arrowstyle='<->', color=WARN, lw=3))
    ax.text(xm - 0.005, (vv.max() + vv.min()) / 2, f'rizado {comma(pp, 1)} mV\n(límite 50 mV)',
            color=WARN, fontsize=26, weight='bold', ha='right', va='center',
            bbox=dict(boxstyle='round,pad=0.3', fc=BG, ec=WARN, lw=2))
    pad = (vv.max() - vv.min()) * 0.6
    ax.set_ylim(vv.min() - pad, vv.max() + pad)
    ax.set_xlabel('tiempo [ms]')
    ax.set_ylabel('tensión [V]')
    note(fig, f'Batería de 7,4 V. Peor caso (batería de 6,2 V): {comma(worst, 1)} mV.')
    return save(fig, '12_rizado')


def bode_data():
    src = (HW / 'spice' / 'radar_amp.cir').read_text()
    if not (SP / 'params.inc').exists():
        raise SystemExit('Falta spice/results/params.inc: corre primero hardware/spice/run_spice.sh')
    with tempfile.TemporaryDirectory() as tmp:
        dat = Path(tmp) / 'bode.dat'
        src = re.sub(r'(ac dec 100 0\.001 10k\n)', rf'\1wrdata {dat} vdb(adc)\n', src)
        src = re.sub(r'^hardcopy .*$', '', src, flags=re.M)
        cir = HW / 'spice' / '.video_bode_tmp.cir'
        cir.write_text(src)
        try:
            subprocess.run(['ngspice', '-b', cir.name], cwd=HW / 'spice', check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        finally:
            cir.unlink(missing_ok=True)
        return read_dat(dat, 1)


def img_bode(vals, f, g):
    gmax, fl, fh = vals[('radar_amp', 'gain_db')], vals[('radar_amp', 'flow')], vals[('radar_amp', 'fhigh')]
    fig = slide(f'Bode: pasa {comma(fl, 1)}–{fh:.0f} Hz', 'R1 · Acondicionamiento · filtro del radar HB100')
    ax = scope_axes(fig)
    ax.semilogx(f, g, color=C1, lw=3)
    ax.axhline(gmax - 3, color=LIMIT, ls='--', lw=2)
    ax.text(5000, gmax - 4.5, '−3 dB', color=LIMIT, fontsize=18, ha='right', va='top')
    for x in (fl, fh):
        ax.axvline(x, color=WARN, ls=':', lw=2)
    ax.axvspan(fl, fh, color=C1, alpha=0.08)
    ax.text(np.sqrt(fl * fh), gmax + 4, f'banda útil {comma(fl, 2)} Hz – {comma(fh, 1)} Hz',
            color=WARN, fontsize=24, weight='bold', ha='center')
    ax.set_xlim(1e-3, 1e4)
    ax.set_ylim(0, gmax + 12)
    ax.set_xlabel('frecuencia [Hz]')
    ax.set_ylabel('ganancia [dB]')
    note(fig, f'Ganancia máxima {comma(gmax, 1)} dB. La señal de prueba de respiración (0,3 Hz) queda dentro de la banda.')
    return save(fig, '13_bode')


def img_motor(vals):
    t, i, w, vm = read_dat(SP / 'motor_start.dat', 3)
    k = t <= 0.4
    ipk = vals[('motor_drive', 'i_peak')]
    fig = slide('Corriente < 1,2 A ✔', 'R1 · Potencia · motor TT con el TB6612 (PWM a duty máximo)')
    ax = scope_axes(fig)
    ax.plot(t[k], i[k], color=C1, lw=1.2, label='corriente del motor')
    ax.axhline(1.2, color=LIMIT, ls='--', lw=2)
    ax.text(0.395, 1.23, 'límite medio del TB6612: 1,2 A', color=LIMIT, ha='right', fontsize=18)
    j = np.argmax(i[k])
    callout(ax, f'pico al arrancar {comma(ipk, 2)} A', (t[k][j], i[k][j]), (0.08, 1.38))
    ax.set_ylim(0, 1.6)
    ax.set_xlabel('tiempo [s]')
    ax.set_ylabel('corriente [A]')
    note(fig, 'La franja gruesa es el rizado del PWM: se ve en detalle en la siguiente imagen.')
    return save(fig, '14_motor_arranque')


def img_pwm(vals):
    t, i, w, vm = read_dat(SP / 'motor_start.dat', 3)
    k = (t >= 0.399) & (t <= 0.4)
    tt, ii = t[k] * 1e3, i[k]
    fig = slide('PWM 20 kHz', 'R1 · Potencia · zoom a 1 ms de la corriente del motor')
    ax = scope_axes(fig)
    ax.plot(tt, ii, color=C1, lw=2.2)
    ax.set_xlabel('tiempo [ms]')
    ax.set_ylabel('corriente [A]')
    pp = vals[('motor_drive', 'ripple_ma')]
    ax.text(0.02, 0.95, f'20 pulsos en 1 ms = 20 kHz · rizado {comma(pp, 0)} mA', transform=ax.transAxes,
            color=WARN, fontsize=24, weight='bold', va='top',
            bbox=dict(boxstyle='round,pad=0.3', fc=BG, ec=WARN, lw=2))
    lo, hi = ii.min(), ii.max()
    ax.set_ylim(lo - 0.6 * (hi - lo), hi + 1.0 * (hi - lo))
    note(fig, 'PWM: el puente H prende y apaga el motor 20 000 veces por segundo para regular su fuerza.')
    return save(fig, '15_motor_pwm')


def img_boot(path, msgs):
    lines = [l for l in msgs[:14] if l.strip()]
    hl = [i for i, l in enumerate(lines) if l.startswith('# findbot firmware')]
    fig = slide('Firmware real en Wokwi', 'R2 · Emulación del ESP32 en Wokwi · monitor serial')
    terminal(fig, lines, highlight=hl, size=22, title=f'docs/evidencia/{path.name}')
    note(fig, 'Arranque del ESP32 emulado con el firmware compilado por PlatformIO (escenario H5 superado, 20/11/2026).')
    return save(fig, '16_wokwi_arranque')


def img_motors_on(path, msgs):
    sel = [l for l in msgs if l.startswith('# findbot') or l.startswith('# motores ON')]
    sel = sel[:2]
    fig = slide('motores ON · PWM 20 kHz', 'R2 · Wokwi · orden de avanzar (v 0.2 0)')
    terminal(fig, ['> v 0.2 0'] + sel, rect=[0.06, 0.62, 0.88, 0.22], highlight=[2], size=24,
             title=f'docs/evidencia/{path.name}')
    ax = show_image(fig, FIG / 'r2_analizador_logico.png', rect=[0.10, 0.22, 0.80, 0.36])
    ax.set_title('Analizador lógico de Wokwi (prueba micro-ROS, 20/11/2026): pines del puente H y del calefactor', fontsize=18,
                 color=MUTED, pad=4)
    return save(fig, '17_wokwi_motores')


def img_encoders(path, msgs):
    sel = [l for l in msgs if l.startswith('# estado')]
    lines = ['> ?'] + [sel[0]] + ['   (se giran las perillas que hacen de encoders)', '> ?', sel[-1]]
    fig = slide('Encoders contando ✔', 'R2 · Wokwi · encoders de cuadratura')
    terminal(fig, lines, highlight=[4], size=19, title=f'docs/evidencia/{path.name}')
    note(fig, 'Encoder: sensor que cuenta las vueltas de cada rueda. enc_l y enc_r pasan de 0 a 20 y −20 pulsos.')
    return save(fig, '18_wokwi_encoders')


def img_watchdog(path, msgs):
    sel = [l for l in msgs if l.startswith('# motores ON') or l.startswith('# vigilante')][:2]
    fig = slide('Modo seguro: frena solo', 'R2 · Máquina de estados de la tracción (firmware)')
    show_image(fig, FIG / 'r2_estados_traccion.png', rect=[0.05, 0.48, 0.90, 0.35],
               boxes=[(900, 2, 1140, 105, LIMIT)])
    terminal(fig, sel, rect=[0.06, 0.24, 0.88, 0.19], highlight=[1], size=24,
             title=f'docs/evidencia/{path.name}  ·  0,5 s sin órdenes')
    return save(fig, '19_wokwi_vigilante')


def img_battery(path, msgs, header, tele, hwcfg):
    it, iv, il = header.index('t_ms'), header.index('vbat'), header.index('bat_level')
    t = np.array([float(p[it]) for p in tele]) / 1000
    v = np.array([float(p[iv]) for p in tele])
    lv = np.array([int(p[il]) for p in tele])
    vlow, vcrit = hwcfg['battery']['v_low'], hwcfg['battery']['v_critical']
    fig = slide('Batería baja → motores OFF', 'R2 · Wokwi · monitor de batería (telemetría del ESP32)')
    ax = scope_axes(fig)
    colors = {0: C1, 1: WARN, 2: LIMIT}
    names = {0: 'nivel normal', 1: 'nivel bajo', 2: 'nivel crítico'}
    for lvl in (0, 1, 2):
        m = lv == lvl
        if m.any():
            ax.scatter(t[m], v[m], s=60, color=colors[lvl], label=names[lvl], zorder=3)
    ax.plot(t, v, color='#6e7681', lw=1.5, zorder=2)
    for y, txt, c in ((vlow, f'aviso {comma(vlow, 1)} V', WARN), (vcrit, f'crítico {comma(vcrit, 1)} V', LIMIT)):
        ax.axhline(y, color=c, ls='--', lw=2)
        ax.text(t.min() + 0.2, y + 0.04, txt, color=c, fontsize=18)
    tc = t[np.argmax(lv == 2)]
    callout(ax, '# motores OFF: bateria critica', (tc, v[np.argmax(lv == 2)]), (tc - 6.5, 6.75), color=LIMIT)
    ax.set_xlabel('tiempo de simulación [s]')
    ax.set_ylabel('tensión de batería medida [V]')
    ax.legend(loc='upper right', fontsize=18)
    note(fig, 'Se baja la perilla que simula la batería: el firmware pasa a nivel bajo, luego a crítico, apaga los motores y el LED parpadea.')
    return save(fig, '20_wokwi_bateria')


def img_montecarlo(vals):
    cols = read_dat(SP / 'stress_mc.dat', 6)
    x = cols[1]
    fig = slide(f'Monte Carlo: {len(x)}/{len(x)} ✔' if (x <= 2.45).all() else 'Monte Carlo', 'R3 · Pruebas de estrés · Monte Carlo')
    ax = scope_axes(fig)
    ax.hist(x, bins=np.arange(2.20, 2.50, 0.01), color=C1, ec=BG)
    ax.axvline(2.45, color=LIMIT, lw=3)
    ax.text(2.445, ax.get_ylim()[1] * 0.92, 'límite fiable del ADC 2,45 V', color=LIMIT, ha='right', fontsize=20)
    callout(ax, f'máximo {comma(x.max(), 3)} V', (x.max(), 3), (2.33, ax.get_ylim()[1] * 0.6))
    ax.set_xlabel('tensión en el ADC con batería llena (8,4 V) [V]')
    ax.set_ylabel('corridas')
    note(fig, f'{len(x)} lotes de componentes sorteados (resistencias ±1 %, condensadores ±10 %): ninguno pasa el límite.')
    return save(fig, '21_montecarlo')


def img_stall(vals):
    t, i = read_dat(SP / 'motor_stall.dat', 1)
    ist = vals[('motor_drive', 'i_stall')]
    fig = slide('Rueda bloqueada < 1,2 A ✔', 'R3 · Pruebas de estrés · peor caso de carga')
    ax = scope_axes(fig)
    ax.plot(t * 1e3, i, color=C1, lw=2)
    ax.axhline(1.2, color=LIMIT, ls='--', lw=2)
    ax.text(t[-1] * 1e3, 1.23, 'límite medio del TB6612: 1,2 A', color=LIMIT, ha='right', fontsize=18)
    callout(ax, f'{comma(ist, 2)} A con la rueda frenada', (t[-1] * 1e3 * 0.8, ist), (t[-1] * 1e3 * 0.35, 0.55))
    ax.set_ylim(0, 1.5)
    ax.set_xlabel('tiempo [ms]')
    ax.set_ylabel('corriente [A]')
    return save(fig, '22_rueda_bloqueada')


def img_table13(n, nf):
    head, rows = md_table('Tabla 13')
    fig = slide(f'{n} pruebas · {nf} fallas ✔', 'R3 · Pruebas de estrés · ¿pasa o no pasa?')
    draw_table(fig, head, rows, [0.13, 0.34, 0.20, 0.20, 0.10], rect=[0.05, 0.22, 0.90, 0.63],
               size=16, wrap=[14, 44, 24, 24, 6], pass_col=4)
    return save(fig, '23_tabla_estres')


def img_table14():
    head, rows = md_table('Tabla 14')
    keep = [0, 1, 3, 4]
    head = [head[k] for k in keep]
    rows = [[r[k] for k in keep] for r in rows]
    fig = slide('4 correcciones al diseño', 'R3 · Lo que la simulación obligó a cambiar')
    draw_table(fig, head, rows, [0.05, 0.36, 0.33, 0.26], rect=[0.05, 0.22, 0.90, 0.63],
               size=21, wrap=[4, 34, 30, 24])
    return save(fig, '24_correcciones')


def img_close(n, nf):
    fig = slide('')
    fig.text(0.5, 0.64, 'Circuito validado ✔', fontsize=80, weight='bold', ha='center', color=ACCENT)
    fig.text(0.5, 0.52, f'{n} criterios de simulación · {nf} fallas · ERC sin errores', fontsize=30,
             ha='center', color=FG)
    fig.text(0.5, 0.44, 'Siguiente paso: comprar los componentes y armar el robot', fontsize=30,
             ha='center', color=MUTED)
    fig.text(0.5, 0.32, 'findbot · UNAD', fontsize=34, weight='bold', ha='center', color=FG)
    return save(fig, '25_cierre')


# ============================================================ guion → subtítulos
def parse_time(s):
    m, sec = s.split(':')
    return int(m) * 60 + float(sec)


def guion_rows():
    """Filas (t0, t1, narración) de las escenas 1–7 del guion."""
    text = (DOCS / 'GUION_VIDEO_HARDWARE.md').read_text()
    rows = []
    for sec in re.split(r'\n## ', text):
        if not re.match(r'[1-7]\. ', sec):
            continue
        for line in sec.splitlines():
            m = re.match(r'\|\s*(\d+:\d+)–(\d+:\d+)\s*\|', line)
            if not m:
                continue
            cells = [c.strip() for c in re.split(r'(?<!\\)\|', line.strip().strip('|'))]
            rows.append((parse_time(m.group(1)), parse_time(m.group(2)), cells[2]))
    return rows


def two_lines(s, maxw=42):
    """Parte un texto en 1 o 2 líneas equilibradas de máx. ~maxw caracteres (None si no cabe)."""
    if len(s) <= maxw:
        return [s]
    spaces = [i for i, ch in enumerate(s) if ch == ' ']
    i = min(spaces, key=lambda i: max(i, len(s) - i - 1))
    left, right = s[:i], s[i + 1:]
    return [left, right] if max(len(left), len(right)) <= maxw + 2 else None


def split_cues(text, maxw=42):
    """Bloques de máx. 2 líneas de ~42 caracteres. Cada frase se reparte en el menor número de
    bloques de largo parecido, cortando de preferencia después de una coma."""
    cues = []
    for sent in re.split(r'(?<=[.?!])\s+', text.strip()):
        n = 1
        while True:
            cuts, start = [], 0
            spaces = [i for i, ch in enumerate(sent) if ch == ' ']
            for k in range(1, n):
                target = k * len(sent) / n
                commas = [i for i in spaces if sent[i - 1] == ',' and abs(i - target) <= 12 and i > start]
                pool = commas or [i for i in spaces if i > start]
                cut = min(pool, key=lambda i: abs(i - target))
                cuts.append(cut)
                start = cut
            parts = [sent[a + 1 if a else 0:b] for a, b in zip([0] + cuts, cuts + [len(sent)])]
            lines = [two_lines(p.strip(), maxw) for p in parts]
            if all(lines):
                cues += ['\n'.join(l) for l in lines]
                break
            n += 1
    return cues


def srt_time(x):
    ms = int(round(x * 1000))
    h, ms = divmod(ms, 3600000)
    m, ms = divmod(ms, 60000)
    s, ms = divmod(ms, 1000)
    return f'{h:02d}:{m:02d}:{s:02d},{ms:03d}'


def write_srt(rows, scale):
    out, n = [], 0
    for t0, t1, text in rows:
        a, b = t0 * scale + 0.2, t1 * scale - 0.2
        cues = split_cues(text)
        total = sum(len(c) for c in cues)
        t = a
        for c in cues:
            d = (b - a) * len(c) / total
            n += 1
            out.append(f'{n}\n{srt_time(t)} --> {srt_time(t + d - 0.05)}\n{c}\n')
            t += d
    SRT.write_text('\n'.join(out), encoding='utf-8')
    return n


# ============================================================ video
def run(cmd):
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if r.returncode:
        print(r.stderr[-3000:])
        raise SystemExit(f'falló: {" ".join(map(str, cmd[:6]))} ...')
    return r


def duration(path):
    r = run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'json', str(path)])
    return float(json.loads(r.stdout)['format']['duration'])


def make_clip(img, frames, k, out):
    """Imagen fija → clip con zoom lento (Ken Burns), alternando acercar y alejar."""
    z = f'1+{ZOOM}*on/{frames}' if k % 2 == 0 else f'{1 + ZOOM}-{ZOOM}*on/{frames}'
    vf = (f'scale={W * 2}:{H * 2}:flags=lanczos,'
          f"zoompan=z='{z}':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d={frames}:s={W}x{H}:fps={FPS},"
          'format=yuv420p')
    run(['ffmpeg', '-y', '-loop', '1', '-framerate', str(FPS), '-i', str(img), '-vf', vf,
         '-frames:v', str(frames), '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', str(out)])


def assemble(shots, scale):
    """Une los clips con fundidos de 0,5 s; el fundido empieza justo en el cambio de toma."""
    BUILD.mkdir(exist_ok=True)
    bounds = [round(s[0] * scale * FPS) for s in shots] + [round(shots[-1][1] * scale * FPS)]
    fade_f = round(FADE * FPS)
    clips = []
    for k, (t0, t1, img) in enumerate(shots):
        seg = bounds[k + 1] - bounds[k]
        frames = seg + (fade_f if k < len(shots) - 1 else 0)
        out = BUILD / f'clip_{k:02d}.mp4'
        make_clip(img, frames, k, out)
        clips.append(out)
        print(f'  clip {k + 1:2d}/{len(shots)}  {img.name:28s} {frames / FPS:5.2f} s')
    total = bounds[-1] / FPS
    inputs, chain, last = [], [], '0:v'
    for c in clips:
        inputs += ['-i', str(c)]
    for k in range(1, len(clips)):
        off = (bounds[k] - bounds[0]) / FPS
        lab = f'v{k}'
        chain.append(f'[{last}][{k}:v]xfade=transition=fade:duration={FADE}:offset={off:.4f}[{lab}]')
        last = lab
    chain.append(f'[{last}]fade=t=in:st=0:d={FADE},fade=t=out:st={total - FADE:.4f}:d={FADE},format=yuv420p[vout]')
    video = BUILD / 'video.mp4'
    run(['ffmpeg', '-y', *inputs, '-filter_complex', ';'.join(chain), '-map', '[vout]',
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-r', str(FPS), str(video)])
    return video, total


def mux(video, total):
    """Versión sin subtítulos (audio: narración o silencio) y versión con subtítulos quemados."""
    if AUDIO.exists():
        a_in = ['-i', str(AUDIO)]
        a_filter = ['-af', f'apad,atrim=0:{total:.3f}']
    else:
        a_in = ['-f', 'lavfi', '-i', 'anullsrc=channel_layout=stereo:sample_rate=48000']
        a_filter = []
    run(['ffmpeg', '-y', '-i', str(video), *a_in, '-map', '0:v', '-map', '1:a', *a_filter,
         '-c:v', 'copy', '-c:a', 'aac', '-b:a', '160k', '-t', f'{total:.3f}',
         '-movflags', '+faststart', str(OUT_SIN)])
    style = ('FontName=DejaVu Sans,FontSize=15,PrimaryColour=&H00FFFFFF,OutlineColour=&H00000000,'
             'BackColour=&H80000000,BorderStyle=1,Outline=2,Shadow=0,Alignment=2,MarginV=12')
    srt = str(SRT).replace('\\', '/').replace(':', '\\:').replace("'", "\\'")
    run(['ffmpeg', '-y', '-i', str(OUT_SIN), '-vf', f"subtitles='{srt}':force_style='{style}'",
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p',
         '-c:a', 'copy', '-movflags', '+faststart', str(OUT_CON)])


def previews(times):
    PREV.mkdir(exist_ok=True)
    for old in PREV.glob('*.png'):
        old.unlink()
    for k, t in enumerate(times, 1):
        run(['ffmpeg', '-y', '-ss', f'{t:.2f}', '-i', str(OUT_CON), '-frames:v', '1',
             str(PREV / f'muestra_{k}_{int(t // 60)}m{int(t % 60):02d}s.png')])


# ============================================================ principal
def main():
    for tool in ('ffmpeg', 'ffprobe', 'ngspice'):
        if not shutil.which(tool):
            raise SystemExit(f'Falta {tool}')
    IMG.mkdir(exist_ok=True)
    for old in IMG.glob('*.png'):
        old.unlink()

    print('1) Imágenes (datos reales del proyecto)')
    vals, ncrit, nfail, stext = summary()
    hwcfg = yaml.safe_load((HW / 'config' / 'robot_hw.yaml').read_text())
    logp, raw, msgs, header, tele = serial_log()
    f, g = bode_data()

    title = img_title()
    e1 = img_r4('02_bloques_energia', 'Energía: 7,4 V → 5 V', (0, 690, 1300, 1060),
                'Batería 2S → interruptor y fusible de 5 A → protección IRF4905 → reguladores MP1584 de 5 V.')
    e2 = img_r4('03_bloques_control', 'Control: ESP32 + puente H', (1440, 580, 3796, 1060),
                'El ESP32 lee la IMU, los encoders y el acondicionamiento analógico, y mueve los motores con el TB6612.')
    e3 = img_r4('04_bloques_usb', 'Comunicación: USB al portátil', (2380, 0, 3796, 520),
                'LiDAR, cámara RGB-D y micrófonos van por un hub USB al portátil (SLAM, Nav2, detección).')
    s0 = img_sch('05_esquematico', 'Esquemático: 5 bloques', 'esquematico_completo.png',
                 'Placa base en KiCad 10, generada desde config/robot_hw.yaml.')
    s1 = img_sch('06_esq_alimentacion', 'Bloque: alimentación', 'esquematico_alimentacion.png',
                 'Batería 2S, interruptor, fusible, protección de polaridad y dos reguladores de 5 V.')
    s2 = img_sch('07_esq_acondicionamiento', 'Bloque: acondicionamiento', 'esquematico_analogico.png',
                 'Amplificadores MCP6004 para el CO₂ y el radar, referencia de 1,30 V y divisor de batería.')
    s3 = img_sch('08_esq_control', 'Bloque: control, potencia y sensores', 'esquematico_esp32.png',
                 'ESP32, puente H TB6612FNG, conectores de motores y encoders, IMU MPU6050.')
    erc = img_erc()
    spi = img_summary(stext, ncrit, nfail)
    pw = img_power(vals)
    rp = img_ripple(vals)
    bd = img_bode(vals, f, g)
    mo = img_motor(vals)
    pm = img_pwm(vals)
    bt = img_boot(logp, msgs)
    mon = img_motors_on(logp, msgs)
    enc = img_encoders(logp, msgs)
    wd = img_watchdog(logp, msgs)
    bat = img_battery(logp, msgs, header, tele, hwcfg)
    mc = img_montecarlo(vals)
    st = img_stall(vals)
    t13 = img_table13(ncrit, nfail)
    t14 = img_table14()
    cl = img_close(ncrit, nfail)
    col = img_collage([(R4, 'Capa física'), (FIG / 'esquematico_completo.png', 'KiCad'),
                       (bd, 'ngspice'), (bat, 'Wokwi')])

    # Tiempos de cada toma sobre la línea de 3:00 del guion (escenas en GUION_VIDEO_HARDWARE.md)
    shots = [
        (0, 8, title), (8, 15, col),                                     # 1 introducción
        (15, 25, e1), (25, 33, e2), (33, 40, e3),                        # 2 diagrama de bloques
        (40, 45, s0), (45, 48, s1), (48, 51, s2), (51, 55, s3), (55, 60, erc),   # 3 KiCad
        (60, 66, spi), (66, 72, pw), (72, 78, rp), (78, 92, bd), (92, 98, mo), (98, 105, pm),  # 4 ngspice
        (105, 113, bt), (113, 120, mon), (120, 128, enc), (128, 136, wd), (136, 145, bat),     # 5 Wokwi
        (145, 152, mc), (152, 157, st), (157, 164, t13), (164, 170, t14),                      # 6 estrés
        (170, 180, cl),                                                  # 7 cierre
    ]

    scale = 1.0
    if AUDIO.exists():
        d = duration(AUDIO)
        scale = min(MAX_S, d + 0.5) / shots[-1][1]
        print(f'   narracion.mp3: {d:.1f} s → escala de tiempos {scale:.3f}')
    else:
        print('   sin narracion.mp3: video sin voz (pista de audio en silencio)')

    print('2) Subtítulos')
    rows = guion_rows()
    ncues = write_srt(rows, scale)
    print(f'   {SRT.name}: {ncues} bloques')

    if '--solo-imagenes' in sys.argv:
        return
    print('3) Clips y montaje')
    video, total = assemble(shots, scale)
    print('4) Versiones final')
    mux(video, total)
    d1, d2 = duration(OUT_SIN), duration(OUT_CON)
    assert abs(d1 - d2) < 0.1, f'duraciones distintas: {d1} vs {d2}'
    assert d1 <= MAX_S + 0.05, f'el video dura {d1} s (> 3:00)'
    previews([t * scale for t in (12, 20, 74, 85, 140, 160)])
    shutil.rmtree(BUILD, ignore_errors=True)
    print(f'\nListo: {d1:.2f} s ({int(d1 // 60)}:{d1 % 60:05.2f})')
    print(f'  {OUT_SIN}\n  {OUT_CON}')


if __name__ == '__main__':
    main()
