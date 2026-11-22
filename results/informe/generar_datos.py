#!/usr/bin/env python3
"""
Datos y gráficas del informe TRL 4, a partir de archivos ya guardados en results/.

No lanza simulaciones ni pruebas: solo lee JSON, CSV, rosbag, ground truth e informes junit.
Uso (dentro del contenedor, con el workspace cargado):
    python3 results/informe/generar_datos.py
"""

import ast
import collections
import csv
import glob
import json
import math
import os
import re
import xml.etree.ElementTree as ElementTree

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from usar_sim_sensors.sound_model import count_obstacles_between  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RESULTS = os.path.join(ROOT, 'results')
CHECK = os.path.join(RESULTS, 'trl4_check_2')
DATA = os.path.join(HERE, 'datos')
FIGS = os.path.join(HERE, 'figuras')
STATION = (-12.0, 0.0)                      # comm_sim.yaml
MISSIONS = [('G0', 'geom_check', 'cámara, dusty 20 m'),
            ('B', 'ablation/hard/B', 'cámara + térmica, hard 30 m'),
            ('D', 'ablation/hard/D', 'cámara + sonido, hard 30 m'),
            ('F', 'ablation/hard/F', 'todos, hard 30 m')]


def read_csv(path):
    with open(path, encoding='utf-8') as stream:
        return [r for r in csv.DictReader(stream) if None not in r.values()]


def write_csv(name, header, rows):
    with open(os.path.join(DATA, name), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(header)
        writer.writerows([[round(v, 3) if isinstance(v, float) else v for v in r] for r in rows])


def only(pattern):
    return sorted(glob.glob(pattern))[0]


def missions():
    rows, series = [], {}
    for key, folder, label in MISSIONS:
        path = only(os.path.join(RESULTS, folder, 'seed*[0-9].json'))
        with open(path, encoding='utf-8') as stream:
            d = json.load(stream)
        ts = read_csv(path[:-5] + '.csv')
        series[key] = (label, ts)
        first = next((float(r['exploration_time']) for r in ts
                      if int(r['victims_detected']) > 0), '')
        rtf = ''
        with open(os.path.join(RESULTS, folder, 'seed1_launch.log'), errors='replace') as stream:
            log = stream.read()
        a = re.search(r'\[(\d+\.\d+)\] \[evaluator\]: Inicio de la expl', log)
        b = re.search(r'\[(\d+\.\d+)\] \[frontier_explorer\]: Exploración terminada', log)
        if a and b:
            rtf = d['exploration_time_s'] / (float(b.group(1)) - float(a.group(1)))
        v, loc = d['victims'], d['robot_localization']
        rows.append([key, label, v['detected'], v['n_victims'], v['detection_rate_pct'],
                     v['false_positives'], v['localization_error_mean_m'], d['coverage_pct'],
                     d['distance_m'], first, loc['slam_error_m_mean'], loc['odom_error_m_mean'],
                     loc['ekf_error_m_mean'], rtf, os.path.relpath(path, ROOT)])
    head = ['mision', 'descripcion', 'detectadas', 'victimas', 'deteccion_pct',
            'falsos_positivos', 'error_victima_m', 'cobertura_pct', 'recorrido_m',
            'primera_victima_s', 'error_slam_m', 'error_odom_m', 'error_ekf_m', 'rtf', 'fuente']
    write_csv('misiones.csv', head, rows)
    summary = []
    for group, chosen in (('hard (B, D, F)', rows[1:]), ('las 4', rows)):
        for i in range(4, 14):
            x = np.array([r[i] for r in chosen if r[i] != ''], dtype=float)
            if len(x) > 1:
                summary.append([group, head[i], len(x), float(x.mean()), float(x.std(ddof=1)),
                                float(x.min()), float(x.max())])
    write_csv('resumen.csv', ['grupo', 'metrica', 'n', 'media', 'desv', 'min', 'max'], summary)
    return series


def read_bag():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(CHECK, 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = collections.defaultdict(list)
    while reader.has_next():
        topic, data, ns = reader.read_next()
        out[topic].append((ns * 1e-9, deserialize_message(data, types[topic])))
    return out


def real_time(bag):
    wall = np.array([t for t, _ in bag['/clock']])
    sim = np.array([m.clock.sec + m.clock.nanosec * 1e-9 for _, m in bag['/clock']])
    to_sim = lambda t: float(np.interp(t, wall, sim))  # noqa: E731
    ctrl = np.array([to_sim(t) for t, _ in bag['/firmware/left_wheel_torque']])
    dt = np.diff(ctrl) * 1000.0
    truth = [m for _, m in bag['/sim/ground_truth/odom']]
    write_csv('tiempo_real.csv', ['metrica', 'valor'], [
        ['rtf_global', float((sim[-1] - sim[0]) / (wall[-1] - wall[0]))],
        ['control_hz', float((len(ctrl) - 1) / (ctrl[-1] - ctrl[0]))],
        ['control_dt_p5_ms', float(np.percentile(dt, 5))],
        ['control_dt_p95_ms', float(np.percentile(dt, 95))],
        ['v_max_m_s', max(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
                          for m in truth)],
        ['w_max_rad_s', max(abs(m.twist.twist.angular.z) for m in truth)]])
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in truth])
    v = np.array([math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for m in truth])
    w = np.array([abs(m.twist.twist.angular.z) for m in truth])
    xy = np.array([(m.pose.pose.position.x, m.pose.pose.position.y) for m in truth])
    return to_sim, t, v, w, xy


def topics(duration):
    groups = collections.defaultdict(list)
    for row in read_csv(only(os.path.join(CHECK, '*_link.csv'))):
        groups[row['topic']].append(row)
    out = []
    for topic, rows in sorted(groups.items()):
        lat = [1000 * (float(r['arrival']) - float(r['sent'])) for r in rows if r['arrival']]
        if lat:
            out.append([topic, len(rows), 100.0 * (1 - len(lat) / len(rows)), len(rows) / duration,
                        len(lat) / duration, float(np.percentile(lat, 50)),
                        float(np.percentile(lat, 95))])
    write_csv('topicos.csv', ['topico', 'enviados', 'perdidos_pct', 'hz_enviados',
                              'hz_entregados', 'latencia_p50_ms', 'latencia_p95_ms'], out)
    return groups['/station/heartbeat']


def link_layers(beats, t, xy):
    with open(os.path.join(ROOT, 'usar_ws', 'worlds', 'seed1_dusty', 'ground_truth.yaml'),
              encoding='utf-8') as stream:
        obstacles = yaml.safe_load(stream)['obstacles']
    by = collections.defaultdict(list)
    for r in beats:
        if not r['rssi_dbm']:
            continue
        p = tuple(xy[min(len(t) - 1, np.searchsorted(t, float(r['sent'])))])
        by[count_obstacles_between(STATION, p, obstacles)].append(
            (math.dist(STATION, p), bool(r['arrival']), float(r['rssi_dbm'])))
    out = [[n, len(a), min(x[0] for x in a), max(x[0] for x in a),
            100.0 * float(np.mean([x[1] for x in a])), float(np.median([x[2] for x in a]))]
           for n, a in sorted(by.items())]
    write_csv('enlace_capas.csv', ['escombros_en_linea', 'n', 'dist_min_m', 'dist_max_m',
                                   'entregados_pct', 'rssi_mediana_dbm'], out)
    return out


def failsafe(bag, to_sim, t, v, w, xy):
    rx = np.array([float(r['received'])
                   for r in read_csv(only(os.path.join(CHECK, '*_heartbeat.csv')))])
    rows, traces = [], []
    for stamp, msg in bag['/link_failsafe/state']:
        if msg.data != 'lost':
            continue
        trip = to_sim(stamp)
        j = np.searchsorted(t, trip)
        if j >= len(t) or (v[j] < 0.05 and w[j] < 0.3):
            continue                        # el robot ya estaba quieto
        last = rx[rx <= trip][-1]
        i = np.searchsorted(t, last)
        # parado: 10 muestras seguidas (0.5 s) sin avance ni giro
        k = next((n for n in range(j, len(t) - 10)
                  if (v[n:n + 10] < 0.02).all() and (w[n:n + 10] < 0.15).all()), None)
        if k is None:
            continue
        kind = 'avance' if v[j] >= 0.05 else 'giro'
        rows.append([float(trip), kind, float(trip - last), float(t[k] - trip),
                     float(t[k] - last),
                     float(np.hypot(*np.diff(xy[i:k + 1], axis=0).T).sum())])
        if kind == 'avance':
            traces.append((t[i:k + 20] - last, v[i:k + 20]))
    write_csv('parada_segura.csv', ['t_s', 'movimiento', 'deteccion_s', 'frenado_s',
                                    'desde_ultimo_latido_s', 'avance_m'], rows)
    return traces


def clearance(obj, x, y):
    """Distancia (m, negativa si está dentro) de un punto a un obstáculo o víctima."""
    dx, dy = x - obj['x'], y - obj['y']
    if obj.get('radius') is not None:
        return math.hypot(dx, dy) - obj['radius']
    yaw = obj.get('yaw', 0.0)
    a = abs(math.cos(yaw) * dx + math.sin(yaw) * dy) - obj.get('size_x', obj.get('length')) / 2
    b = abs(-math.sin(yaw) * dx + math.cos(yaw) * dy) - obj.get('size_y', obj.get('width')) / 2
    return math.hypot(max(a, 0), max(b, 0)) if a > 0 or b > 0 else max(a, b)


def collisions():
    # contorno del robot (xacro): chasis y extremos de las ruedas
    outline = [(x, y) for x in np.linspace(-0.16, 0.06, 12) for y in (-0.08, 0.08)]
    outline += [(x, y) for x in (-0.16, 0.06) for y in np.linspace(-0.08, 0.08, 9)]
    outline += [(x, y) for x in (-0.0325, 0.0, 0.0325) for y in (-0.111, 0.111)]
    out = []
    for key, folder, world in (('B', 'ablation/hard/B', 'seed1_hard'),
                               ('D', 'ablation/hard/D', 'seed1_hard'),
                               ('F', 'ablation/hard/F', 'seed1_hard'),
                               ('T', 'trl4_check_2', 'seed1_dusty')):
        with open(os.path.join(ROOT, 'usar_ws', 'worlds', world, 'ground_truth.yaml'),
                  encoding='utf-8') as stream:
            gt = yaml.safe_load(stream)
        poses = read_csv(only(os.path.join(RESULTS, folder, '*_pose.csv')))
        low, hits = math.inf, 0
        for p in poses:
            x, y, h = float(p['true_x']), float(p['true_y']), float(p['true_yaw'])
            near = [o for o in gt['obstacles'] + gt['victims']
                    if math.hypot(x - o['x'], y - o['y']) < 3.0]
            here = min([clearance(o, x + a * math.cos(h) - b * math.sin(h),
                                  y + a * math.sin(h) + b * math.cos(h))
                        for o in near for a, b in outline] or [math.inf])
            low, hits = min(low, here), hits + (here <= 0)
        out.append([key, len(poses), float(low), hits])
    write_csv('colisiones.csv', ['mision', 'muestras_1hz', 'separacion_min_m', 'contactos'], out)


def maintainability():
    src, keys = os.path.join(ROOT, 'usar_ws', 'src'), set()

    def walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                keys.add(str(key))
                walk(value)
    for path in glob.glob(os.path.join(src, '*', 'config', '*.yaml')):
        with open(path, encoding='utf-8') as stream:
            walk(yaml.safe_load(stream))
    runtime = {'ground_truth_file', 'results_dir', 'result_prefix', 'area_size', 'log_csv'}
    tunable = in_yaml = code = tests = 0
    for path in glob.glob(os.path.join(src, '*', '**', '*.py'), recursive=True):
        with open(path, encoding='utf-8') as stream:
            text = stream.read()
        lines = sum(1 for line in text.splitlines() if line.strip())
        if '/test/' in path:
            tests += lines
            continue
        code += lines
        for node in ast.parse(text).body:
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict) and any(
                    getattr(t, 'id', '') in ('PARAMS', 'DEFAULTS') for t in node.targets):
                for key in node.value.keys:
                    if key is not None and key.value not in runtime:
                        tunable += 1
                        in_yaml += key.value in keys
    write_csv('mantenibilidad.csv', ['metrica', 'valor'], [
        ['paquetes_ros', len(glob.glob(os.path.join(src, '*', 'package.xml')))],
        ['lineas_python', code], ['lineas_pruebas_python', tests],
        ['parametros_ajustables', tunable], ['parametros_en_yaml', in_yaml],
        ['documentos_fase', len(glob.glob(os.path.join(ROOT, 'docs', '*.md')))]])


def software_tests():
    out, build = [], os.path.join(ROOT, 'usar_ws', 'build')
    paths = sorted(glob.glob(os.path.join(build, '*', 'pytest.xml')) + glob.glob(
        os.path.join(build, 'usar_firmware_sil', 'test_results', '*', '*.gtest.xml')))
    for path in paths:
        cases = list(ElementTree.parse(path).getroot().iter('testcase'))
        failed = sum(c.find('failure') is not None for c in cases)
        skipped = sum(c.find('skipped') is not None for c in cases)
        out.append([os.path.relpath(path, build).split(os.sep)[0], len(cases),
                    len(cases) - failed - skipped, failed, skipped])
    out.append(['total'] + [sum(r[i] for r in out) for i in range(1, 5)])
    write_csv('pruebas_software.csv', ['paquete', 'casos', 'pasan', 'fallan', 'omitidos'], out)


def figures(series, layers, traces):
    plt.rcParams.update({'font.size': 8})
    fig, (a, b, c) = plt.subplots(1, 3, figsize=(8.4, 2.35), dpi=220)
    for key, (label, ts) in series.items():
        a.plot([float(r['exploration_time']) for r in ts], [float(r['coverage_pct']) for r in ts],
               label=f'{key}: {label}')
    a.axhline(90, color='k', ls=':', lw=0.8)
    a.set(title='(a) Cobertura', xlabel='Tiempo simulado (s)', ylabel='Área explorada (%)',
          ylim=(0, 103))
    a.legend(fontsize=5, loc='lower right')
    bars = b.bar([str(r[0]) for r in layers], [r[4] for r in layers], color='tab:blue')
    b.bar_label(bars, [f'{r[2]:.0f}–{r[3]:.0f} m' for r in layers], label_type='center',
                color='k', fontsize=6)
    b.set(title='(b) Enlace con la estación', xlabel='Escombros en la línea',
          ylabel='Latidos entregados (%)', ylim=(0, 105))
    for i, (x, y) in enumerate(traces, 1):
        c.plot(x, y, label=f'evento {i}')
    c.axvline(1.0, color='r', ls=':', lw=0.8, label='objetivo (1.0 s)')
    c.set(title='(c) Parada segura', xlabel='Tiempo desde el último latido (s)',
          ylabel='Velocidad real (m/s)')
    c.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, 'fig01_resultados.png'))
    plt.close(fig)

    # Mapa de la misión B tal como lo dibujó el evaluador, sin el título.
    image = plt.imread(only(os.path.join(RESULTS, 'ablation', 'hard', 'B', '*_comparison.png')))
    plt.imsave(os.path.join(FIGS, 'fig02_mapa_mision_b.png'), image[88:])


def main():
    os.makedirs(DATA, exist_ok=True)
    os.makedirs(FIGS, exist_ok=True)
    series = missions()
    bag = read_bag()
    to_sim, t, v, w, xy = real_time(bag)
    beats = topics(float(t[-1] - t[0]))
    layers = link_layers(beats, t, xy)
    figures(series, layers, failsafe(bag, to_sim, t, v, w, xy))
    software_tests()
    collisions()
    maintainability()


if __name__ == '__main__':
    main()
