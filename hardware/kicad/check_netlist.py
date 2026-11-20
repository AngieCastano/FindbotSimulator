#!/usr/bin/env python3
"""Exporta la netlist del esquemático y la compara con config/robot_hw.yaml.

Verifica: (1) cada GPIO de la tabla de pines está en la red con su nombre de función,
(2) ningún GPIO prohibido o libre quedó conectado, (3) toda red con nombre tiene ≥ 2 pines,
(4) los valores de los componentes analógicos son los simulados en la H2.
Uso: python3 hardware/kicad/check_netlist.py   (código de salida 1 si algo no coincide)
"""
import subprocess
import sys
from pathlib import Path

import yaml

from gen_schematic import HW_YAML, KICAD_DIR, num, si
from sexpr import find, find_all, parse

OUT = KICAD_DIR / 'out'


def load_netlist():
    """Exporta con kicad-cli y devuelve ({red: [(ref, pin, nombre_pin)]}, {ref: valor})."""
    OUT.mkdir(exist_ok=True)
    path = OUT / 'findbot.net'
    subprocess.run(['kicad-cli', 'sch', 'export', 'netlist', '--format', 'kicadsexpr',
                    '-o', str(path), str(KICAD_DIR / 'findbot.kicad_sch')],
                   check=True, capture_output=True)
    root = parse(path.read_text(encoding='utf-8'))
    nets = {}
    for net in find_all(find(root, 'nets'), 'net'):
        name = str(find(net, 'name')[1]).lstrip('/')
        nodes = []
        for node in find_all(net, 'node'):
            pinfn = find(node, 'pinfunction')
            nodes.append((str(find(node, 'ref')[1]), str(find(node, 'pin')[1]),
                          str(pinfn[1]) if pinfn else ''))
        nets[name] = nodes
    values = {str(find(c, 'ref')[1]): str(find(c, 'value')[1])
              for c in find_all(find(root, 'components'), 'comp')}
    return nets, values


def main():
    cfg = yaml.safe_load(HW_YAML.read_text(encoding='utf-8'))
    nets, values = load_netlist()
    errors = []

    esp_pin_net = {}
    for name, nodes in nets.items():
        for ref, _, pinfn in nodes:
            if ref == 'U3':
                esp_pin_net[pinfn] = name
    for pin in cfg['pins']:
        io = next((k for k in esp_pin_net if k.split('_')[0] == f'IO{pin["gpio"]}'), None)
        if pin.get('onboard'):
            if io and not esp_pin_net[io].startswith('unconnected'):
                errors.append(f'GPIO{pin["gpio"]} es de la placa DevKit y no debería conectarse')
            continue
        got = esp_pin_net.get(io)
        if got != pin['func']:
            errors.append(f'GPIO{pin["gpio"]}: se esperaba la red {pin["func"]}, está en {got}')
    used = {f'IO{p["gpio"]}' for p in cfg['pins'] if not p.get('onboard')}
    for io, net in esp_pin_net.items():
        if io.startswith('IO') and io.split('_')[0] not in used and not net.startswith('unconnected'):
            errors.append(f'{io.split("_")[0]} no está en la tabla de pines pero está conectado a {net}')

    for name, nodes in nets.items():
        if not name.startswith('unconnected') and len(nodes) < 2:
            errors.append(f'la red {name} tiene un solo pin')

    an, bat = cfg['analog'], cfg['battery']['divider']
    expected = {
        'R14': si(an['co2']['gain_rf']), 'R15': si(an['co2']['gain_rg']),
        'R21': si(an['radar']['rin']), 'R22': si(an['radar']['rf']), 'C11': si(num(an['radar']['cf'])),
        'C10': si(num(an['radar']['cin'])), 'R26': si(bat['r_top']), 'R27': si(bat['r_bottom']),
        'C15': si(num(bat['c_filter'])), 'R4': si(an['encoder']['pullup']),
    }
    for ref, val in expected.items():
        if values.get(ref) != val:
            errors.append(f'{ref}: valor {values.get(ref)} distinto del simulado ({val})')

    named = [n for n in nets if not n.startswith('unconnected')]
    for err in errors:
        print(f'ERROR  {err}')
    print(f'{len(named)} redes, {len(values)} componentes, '
          f'{len(cfg["pins"])} GPIO revisados, {len(errors)} errores')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
