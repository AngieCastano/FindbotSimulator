#!/usr/bin/env python3
"""Coherencia entre el hardware y el resto del proyecto (Fase H7).

ERRORES (zona hardware, exit 1):
  - tabla de pines de hardware/CLAUDE.md ≠ robot_hw.yaml
  - tabla de geometría de hardware/CLAUDE.md ≠ robot_hw.yaml
  - un tópico MVP conectado al ESP32 en la tabla del CLAUDE.md raíz que el firmware no publica
ADVERTENCIAS (zona simulación o fase 2; solo se LEEN esos archivos, nunca se modifican):
  - geometría y límites de usar_description/config/robot.yaml distintos de robot_hw.yaml
  - tópicos del firmware ausentes o con otro tipo en bridge.yaml; frames/joints ausentes del xacro
  - sensores de fase 2 del ESP32 que el firmware aún no publica
Uso: python3 hardware/scripts/check_consistency.py
"""
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
HW = ROOT / 'hardware'
SIM = ROOT / 'usar_ws' / 'src' / 'usar_description'
TOL = 1e-4

# Tipos que publica/suscribe el firmware (ros_bridge.cpp)
FW_TYPES = {'cmd_vel': 'geometry_msgs/msg/Twist', 'odom': 'nav_msgs/msg/Odometry',
            'joint_states': 'sensor_msgs/msg/JointState', 'imu': 'sensor_msgs/msg/Imu',
            'battery': 'sensor_msgs/msg/BatteryState', 'tf': 'tf2_msgs/msg/TFMessage'}


def num(text):
    """'−0,05' → -0.05 (formato de las tablas en español)."""
    return float(text.strip().replace('−', '-').replace(',', '.'))


def md_rows(text, first_cell_re):
    """Filas de tablas markdown cuya primera celda cumple la expresión."""
    rows = []
    for line in text.splitlines():
        if line.startswith('|'):
            # quitar solo el primer y el último '|' (una celda final vacía debe conservarse)
            cells = [c.strip() for c in line.strip()[1:-1].split('|')]
            if cells and re.fullmatch(first_cell_re, cells[0]):
                rows.append(cells)
    return rows


def check_pin_table(cfg, md, errors):
    """hardware/CLAUDE.md, tabla de pines: '| 18 / 19 | `ENC_L_A` / `ENC_L_B` | ...'."""
    doc = {}
    for cells in md_rows(md, r'\d+( / \d+)?'):
        gpios = [int(g) for g in cells[0].split('/')]
        funcs = re.findall(r'`([A-Z0-9_]+)`', cells[1])
        if len(gpios) == len(funcs):
            doc.update(dict(zip(funcs, gpios)))
    yaml_pins = {p['func']: p['gpio'] for p in cfg['pins']}
    for func, gpio in yaml_pins.items():
        if doc.get(func) != gpio:
            errors.append(f'hardware/CLAUDE.md: {func} en GPIO{doc.get(func)}, robot_hw.yaml dice GPIO{gpio}')
    for func in set(doc) - set(yaml_pins):
        errors.append(f'hardware/CLAUDE.md: {func} no está en robot_hw.yaml')


def check_geometry_table(cfg, md, errors):
    """hardware/CLAUDE.md, tabla de geometría: columnas Frame | x | y | z | z suelo."""
    sensors = {s['frame']: s for s in cfg['geometry']['sensors'].values()}
    seen = 0
    for cells in md_rows(md, r'[^|]+'):
        frame = re.findall(r'`([a-z0-9_]+)`', cells[1]) if len(cells) > 6 else []
        if not frame or frame[0] not in sensors:
            continue
        seen += 1
        s = sensors[frame[0]]
        for col, key in ((2, 'x'), (3, 'y'), (4, 'z'), (5, 'z_ground')):
            try:
                value = num(cells[col])
            except ValueError:
                continue
            if abs(value - float(s[key])) > TOL:
                errors.append(f'hardware/CLAUDE.md: {frame[0]}.{key} = {value}, robot_hw.yaml = {s[key]}')
    if seen < len(sensors):
        errors.append(f'hardware/CLAUDE.md: la tabla de geometría tiene {seen} de {len(sensors)} sensores')


def check_root_table(cfg, warnings, errors):
    """Tabla de sensores del CLAUDE.md raíz: lo que va al ESP32 debe estar en el firmware."""
    root_md = (ROOT / 'CLAUDE.md').read_text(encoding='utf-8')
    fw_topics = set(cfg['firmware']['ros']['topics'].values())
    for cells in md_rows(root_md, r'\d+'):
        if len(cells) < 8 or 'ESP32' not in cells[3]:
            continue
        element, state = cells[1], cells[7]
        topics = re.findall(r'`(/[a-z0-9_/]+)`', cells[5])
        missing = [t for t in topics if t not in fw_topics]
        if not missing:
            continue
        msg = f'CLAUDE.md raíz, fila {cells[0]} ({element}): el firmware no publica {", ".join(missing)}'
        (errors if state == 'MVP' else warnings).append(msg + ('' if state == 'MVP' else f' [{state}]'))


def check_sim(cfg, warnings):
    """Archivos de la simulación (solo lectura)."""
    robot = SIM / 'config' / 'robot.yaml'
    if not robot.exists():
        warnings.append(f'no existe {robot}')
        return
    sim = yaml.safe_load(robot.read_text(encoding='utf-8'))
    geo = cfg['geometry']
    pairs = [
        ('wheels.radius', sim['wheels']['radius'], geo['wheels']['radius']),
        ('wheels.width', sim['wheels']['width'], geo['wheels']['width']),
        ('wheels.separation', sim['wheels']['separation'], geo['wheels']['separation']),
        ('chassis.length', sim['chassis']['length'], geo['chassis']['length']),
        ('chassis.width', sim['chassis']['width'], geo['chassis']['width']),
        ('chassis.height', sim['chassis']['height'], geo['chassis']['height']),
        ('chassis.x_offset', sim['chassis']['x_offset'], geo['chassis']['x_offset']),
        ('chassis.z_offset', sim['chassis']['z_offset'], geo['chassis']['z_offset']),
        ('chassis.mass', sim['chassis']['mass'], geo['chassis']['mass']),
        ('caster.radius', sim['caster']['radius'], geo['caster']['radius']),
        ('caster.x_offset', sim['caster']['x_offset'], geo['caster']['x_offset']),
        ('lidar.x', sim['lidar']['x'], geo['sensors']['lidar']['x']),
        ('lidar.z', sim['lidar']['z'], geo['sensors']['lidar']['z']),
        ('lidar.min_range', sim['lidar']['min_range'], geo['sensors']['lidar']['min_range']),
        ('camera.x', sim['camera']['x'], geo['sensors']['camera']['x']),
        ('camera.z', sim['camera']['z'], geo['sensors']['camera']['z']),
        ('camera.near', sim['camera']['near'], geo['sensors']['camera']['min_depth']),
        ('diff_drive.max_linear_velocity', sim['diff_drive']['max_linear_velocity'],
         geo['limits']['max_linear_velocity']),
        ('diff_drive.max_angular_velocity', sim['diff_drive']['max_angular_velocity'],
         geo['limits']['max_angular_velocity']),
    ]
    for key in ('co2', 'thermal', 'imu', 'radar', 'mic_array'):
        if key in sim and key in geo['sensors']:
            for axis in ('x', 'y', 'z'):
                if axis in sim[key]:
                    pairs.append((f'{key}.{axis}', sim[key][axis], geo['sensors'][key][axis]))
        elif key in geo['sensors'] and key not in ('radar', 'mic_array'):
            warnings.append(f'robot.yaml (sim): falta el bloque "{key}" (posición en hardware/CLAUDE.md)')
    for name, sim_v, hw_v in pairs:
        if abs(float(sim_v) - float(hw_v)) > TOL:
            warnings.append(f'robot.yaml (sim): {name} = {sim_v}, hardware = {hw_v}')

    bridge = yaml.safe_load((SIM / 'config' / 'bridge.yaml').read_text(encoding='utf-8'))
    btypes = {b['ros_topic_name']: b['ros_type_name'] for b in bridge}
    for key, topic in cfg['firmware']['ros']['topics'].items():
        if key == 'battery':
            continue  # la batería no existe en Gazebo ("no aplica" en la tabla raíz)
        if topic not in btypes:
            warnings.append(f'bridge.yaml (sim): falta {topic} [{FW_TYPES[key]}] que publica el robot real')
        elif btypes[topic] != FW_TYPES[key]:
            warnings.append(f'bridge.yaml (sim): {topic} es {btypes[topic]}, el firmware usa {FW_TYPES[key]}')

    xacro = (SIM / 'urdf' / 'findbot.urdf.xacro').read_text(encoding='utf-8')
    ros = cfg['firmware']['ros']
    names = set(re.findall(r'<(?:link|joint) name="([^"]+)"', xacro))
    names |= {n.replace('${side}', side) for n in names for side in ('left', 'right')}
    # 'odom' no es un link del URDF: es el frame fijo que crea la TF odom → base_footprint
    for frame in [f for k, f in ros['frames'].items() if k != 'odom'] + list(ros['joints']):
        if frame not in names:
            warnings.append(f'xacro (sim): falta "{frame}" (frame/joint que usa el firmware)')


def main():
    cfg = yaml.safe_load((HW / 'config' / 'robot_hw.yaml').read_text(encoding='utf-8'))
    md = (HW / 'CLAUDE.md').read_text(encoding='utf-8')
    errors, warnings = [], []
    check_pin_table(cfg, md, errors)
    check_geometry_table(cfg, md, errors)
    check_root_table(cfg, warnings, errors)
    check_sim(cfg, warnings)
    for e in errors:
        print(f'ERROR        {e}')
    for w in warnings:
        print(f'ADVERTENCIA  {w}')
    print(f'{len(errors)} errores (zona hardware), {len(warnings)} advertencias (simulación / fase 2)')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
