#!/usr/bin/env python3
"""Verifica la asignación de pines y la geometría de hardware/config/robot_hw.yaml.

Reglas del ESP32-WROOM-32: sin pines de flash ni de UART0, sin pines de arranque (salvo los
permitidos), entradas analógicas solo en ADC1, salidas nunca en pines de solo entrada, sin
pines repetidos. Reglas de geometría: nada por encima del plano del LiDAR.
Uso: python3 hardware/scripts/check_pins.py [ruta_yaml]   (código de salida 1 si hay errores)
"""
import sys
from pathlib import Path

import yaml

OUTPUT_MODES = {'out', 'pwm'}


def check(cfg):
    """Devuelve la lista de errores encontrados (vacía si todo está bien)."""
    rules = cfg['esp32_rules']
    errors = []
    seen = {}
    for pin in cfg['pins']:
        gpio, func, mode = pin['gpio'], pin['func'], pin['mode']
        if gpio not in rules['valid']:
            errors.append(f'{func}: GPIO{gpio} no existe en el ESP32')
        if gpio in rules['forbidden']:
            errors.append(f'{func}: GPIO{gpio} está reservado (flash o UART0 de micro-ROS)')
        if gpio in rules['strapping'] and gpio not in rules['strapping_allowed']:
            errors.append(f'{func}: GPIO{gpio} es pin de arranque (strapping)')
        if gpio in rules['input_only'] and mode in OUTPUT_MODES:
            errors.append(f'{func}: GPIO{gpio} es solo entrada y se usa como salida')
        if mode == 'adc' and gpio not in rules['adc1']:
            errors.append(f'{func}: GPIO{gpio} no es ADC1 (ADC2 falla con WiFi)')
        if mode == 'in_quadrature' and gpio in rules['input_only']:
            errors.append(f'{func}: GPIO{gpio} no tiene pull-up interno para el encoder')
        if gpio in seen:
            errors.append(f'{func}: GPIO{gpio} ya lo usa {seen[gpio]}')
        seen[gpio] = func

    geo = cfg['geometry']
    lidar_plane = geo['sensors']['lidar']['z_ground']
    for name, sensor in geo['sensors'].items():
        if name != 'lidar' and sensor['z_ground'] >= lidar_plane:
            errors.append(f'{name}: z_ground={sensor["z_ground"]} tapa el plano del LiDAR')
    if geo['chassis']['upper_deck_top_z_ground'] >= lidar_plane:
        errors.append('la placa alta queda por encima del plano del LiDAR')

    div = cfg['battery']['divider']
    v_adc_max = cfg['battery']['v_full'] * div['r_bottom'] / (div['r_top'] + div['r_bottom'])
    if v_adc_max > 2.45:
        errors.append(f'divisor de batería: {v_adc_max:.2f} V en el ADC supera 2.45 V')
    return errors


def main():
    """Punto de entrada por línea de comandos."""
    default = Path(__file__).resolve().parents[1] / 'config' / 'robot_hw.yaml'
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))
    errors = check(cfg)
    for err in errors:
        print(f'ERROR  {err}')
    print(f'{len(cfg["pins"])} pines revisados, {len(errors)} errores')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
