#!/usr/bin/env python3
"""Genera el esquemático de la placa base del robot (KiCad) desde config/robot_hw.yaml.

Salida en hardware/kicad/: findbot.kicad_sch, findbot.kicad_sym (símbolos propios de los
módulos), findbot.kicad_pro y sym-lib-table. El esquemático usa etiquetas de red en cada pin
(sin cables largos): las redes de los GPIO salen de la tabla de pines del YAML, así que el
esquemático y la tabla no pueden quedar distintos. ESTE SCRIPT ES LA FUENTE: los cambios
hechos a mano en KiCad se pierden al regenerar.
Uso: python3 hardware/kicad/gen_schematic.py
"""
import copy
import math
import uuid
from pathlib import Path

import yaml

from sexpr import Q, dump, find, find_all, parse

KICAD_DIR = Path(__file__).resolve().parent
HW_YAML = KICAD_DIR.parent / 'config' / 'robot_hw.yaml'
SYS_SYMBOLS = Path('/usr/share/kicad/symbols')
PROJECT = 'findbot'
GRID = 2.54
NS = uuid.UUID('6f1d1c52-5a51-4f8e-9d5e-0b7e8f1d2a10')

FP = {
    'R': 'Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal',
    'C': 'Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm',
    'C_SMD': 'Capacitor_SMD:C_1206_3216Metric',
    'CP_big': 'Capacitor_THT:CP_Radial_D10.0mm_P5.00mm',
    'CP': 'Capacitor_THT:CP_Radial_D6.3mm_P2.50mm',
    'L': 'Inductor_THT:L_Radial_D10.0mm_P5.00mm_Fastron_07M',
    'LED': 'LED_THT:LED_D3.0mm',
    'FUSE': 'Fuse:Fuseholder_Blade_Mini_Keystone_3568',
    'SW': 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal',
    'XT30_M': 'Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal',
    'XT30_F': 'Connector_AMASS:AMASS_XT30PW-F_1x02_P2.50mm_Horizontal',
    'TO220': 'Package_TO_SOT_THT:TO-220-3_Vertical',
    'DIP14': 'Package_DIP:DIP-14_W7.62mm',
    'XH3': 'Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical',
    'XH4': 'Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical',
    'XH6': 'Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical',
    'SOCK4': 'Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical',
    'SOCK8': 'Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical',
    'HDR4': 'Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical',
    'POLOLU16': 'Module:Pololu_Breakout-16_15.2x20.3mm',
}


def uid(*key):
    """UUID determinista: regenerar no cambia los identificadores (diffs limpios)."""
    return Q(str(uuid.uuid5(NS, ':'.join(str(k) for k in key))))


def si(value):
    """Formatea un valor con prefijo SI al estilo KiCad (27000 → 27k, 1e-7 → 100n)."""
    value = float(value)
    for exp, prefix in ((6, 'M'), (3, 'k'), (0, ''), (-3, 'm'), (-6, 'u'), (-9, 'n'), (-12, 'p')):
        scaled = value / 10 ** exp
        if abs(scaled) >= 1:
            text = f'{scaled:.3g}'
            return text.replace('.', prefix) if prefix and '.' in text else text + prefix
    return str(value)


def num(value):
    """Lee números que PyYAML deja como texto (p. ej. 1.0e-5)."""
    return float(value)


# ------------------------------------------------------------------------------------------
# Símbolos propios (módulos) → findbot.kicad_sym
# ------------------------------------------------------------------------------------------
def font():
    return ['effects', ['font', ['size', 1.27, 1.27]]]


def hidden():
    return ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]


def box_symbol(name, ref, value, left, right, desc, footprint=''):
    """Símbolo rectangular con pines (número, nombre, tipo) a la izquierda y a la derecha."""
    rows = max(len(left), len(right))
    half_w = 12.7
    half_h = (rows - 1) / 2 * GRID + GRID
    pins = []
    for side, plist in ((-1, left), (1, right)):
        for i, (pnum, pname, ptype) in enumerate(plist):
            y = ((rows - 1) / 2 - i) * GRID
            x = side * (half_w + GRID)
            angle = 0 if side < 0 else 180
            pins.append(['pin', ptype, 'line', ['at', x, y, angle], ['length', GRID],
                         ['name', Q(pname), font()], ['number', Q(str(pnum)), font()]])
    return ['symbol', Q(name), ['pin_names', ['offset', 1.016]],
            ['exclude_from_sim', 'no'], ['in_bom', 'yes'], ['on_board', 'yes'],
            ['property', Q('Reference'), Q(ref), ['at', 0, half_h + 1.27, 0], font()],
            ['property', Q('Value'), Q(value), ['at', 0, -half_h - 1.27, 0], font()],
            ['property', Q('Footprint'), Q(footprint), ['at', 0, 0, 0], hidden()],
            ['property', Q('Datasheet'), Q(''), ['at', 0, 0, 0], hidden()],
            ['property', Q('Description'), Q(desc), ['at', 0, 0, 0], hidden()],
            ['symbol', Q(f'{name}_0_1'),
             ['rectangle', ['start', -half_w, half_h], ['end', half_w, -half_h],
              ['stroke', ['width', 0.254], ['type', 'default']], ['fill', ['type', 'background']]]],
            ['symbol', Q(f'{name}_1_1'), *pins],
            ['embedded_fonts', 'no']]


IO, PIN_IN, PIN_OUT, PWR_IN, PWR_OUT, PAS = (
    'bidirectional', 'input', 'output', 'power_in', 'power_out', 'passive')

# ESP32 DOIT DevKit V1 (30 pines). Numeración: 1–15 fila izquierda (antena arriba), 16–30 derecha.
DEVKIT_LEFT = ['EN', 'IO36', 'IO39', 'IO34', 'IO35', 'IO32', 'IO33', 'IO25', 'IO26', 'IO27',
               'IO14', 'IO12', 'IO13', 'GND', 'VIN']
DEVKIT_RIGHT = ['IO23', 'IO22', 'IO1_TX0', 'IO3_RX0', 'IO21', 'IO19', 'IO18', 'IO5', 'IO17',
                'IO16', 'IO4', 'IO2', 'IO15', 'GND', '3V3']


def devkit_type(name):
    return {'3V3': PWR_OUT, 'GND': PWR_IN, 'VIN': PWR_IN, 'EN': PIN_IN}.get(name, IO)


def custom_symbols():
    """Define los símbolos propios de la placa base."""
    left = [(i + 1, n, devkit_type(n)) for i, n in enumerate(DEVKIT_LEFT)]
    right = [(i + 16, n, devkit_type(n)) for i, n in enumerate(DEVKIT_RIGHT)]
    return [
        box_symbol('ESP32_DevKitV1_30', 'U', 'ESP32 DevKit V1', left, right,
                   'ESP32-WROOM-32 DOIT DevKit V1, 30 pines, sobre 2 zócalos 1x15 '
                   '(separación 22,86 mm, verificar). Se alimenta por su USB desde el hub.'),
        box_symbol('TB6612_Module', 'U', 'TB6612FNG módulo',
                   [(1, 'GND', PWR_IN), (2, 'VCC', PWR_IN), (3, 'AO1', PIN_OUT), (4, 'AO2', PIN_OUT),
                    (5, 'BO2', PIN_OUT), (6, 'BO1', PIN_OUT), (7, 'VMOT', PWR_IN), (8, 'GND', PWR_IN)],
                   [(9, 'PWMA', PIN_IN), (10, 'AIN2', PIN_IN), (11, 'AIN1', PIN_IN), (12, 'STBY', PIN_IN),
                    (13, 'BIN1', PIN_IN), (14, 'BIN2', PIN_IN), (15, 'PWMB', PIN_IN), (16, 'GND', PWR_IN)],
                   'Módulo TB6612FNG tipo Pololu #713 (VERIFICAR el orden de pines con la placa comprada)',
                   FP['POLOLU16']),
        box_symbol('GY521_Module', 'U', 'GY-521 (MPU6050)',
                   [(1, 'VCC', PWR_IN), (2, 'GND', PWR_IN), (3, 'SCL', IO), (4, 'SDA', IO)],
                   [(5, 'XDA', IO), (6, 'XCL', IO), (7, 'AD0', PIN_IN), (8, 'INT', PIN_OUT)],
                   'Módulo GY-521 con MPU6050 (IMU 6 ejes, I2C 0x68)', FP['SOCK8']),
        box_symbol('Buck_Module', 'U', 'Buck 5 V',
                   [(1, 'IN+', PWR_IN), (2, 'IN-', PAS)], [(3, 'OUT+', PWR_OUT), (4, 'OUT-', PAS)],
                   'Módulo reductor MP1584EN ajustado a 5,0 V', FP['HDR4']),
        box_symbol('TGS4161', 'J', 'TGS4161',
                   [(1, 'H1', PAS), (2, 'H2', PAS)], [(3, 'S+', PAS), (4, 'S-', PAS)],
                   'Sensor de CO2 Figaro TGS4161 por cable a conector XH4 (H = calefactor, S = FEM)',
                   FP['XH4']),
    ]


# ------------------------------------------------------------------------------------------
# Bibliotecas: carga y aplanado de símbolos derivados (extends)
# ------------------------------------------------------------------------------------------
_LIB_CACHE = {}


def lib_node(lib):
    if lib not in _LIB_CACHE:
        path = KICAD_DIR / f'{PROJECT}.kicad_sym' if lib == PROJECT else SYS_SYMBOLS / f'{lib}.kicad_sym'
        _LIB_CACHE[lib] = parse(path.read_text(encoding='utf-8'))
    return _LIB_CACHE[lib]


ORDER = ['pin_numbers', 'pin_names', 'exclude_from_sim', 'in_bom', 'on_board', 'in_pos_files',
         'duplicate_pin_numbers_are_jumpers',
         'property', 'symbol', 'embedded_fonts']


def flat_symbol(lib, name):
    """Devuelve el símbolo sin 'extends' (las unidades gráficas vienen del padre)."""
    node = next(s for s in find_all(lib_node(lib), 'symbol') if s[1] == name)
    ext = find(node, 'extends')
    if not ext:
        return copy.deepcopy(node)
    parent = flat_symbol(lib, ext[1])
    own = [x for x in node[2:] if not (isinstance(x, list) and x[0] == 'extends')]
    own_keys = {x[0] for x in own if isinstance(x, list)}
    merged = [x for x in parent[2:] if isinstance(x, list)
              and x[0] not in ('property', 'symbol') and x[0] not in own_keys]
    # Propiedades del padre que el hijo no redefine (p. ej. ki_locked) se heredan
    own_props = {p[1] for p in find_all(node, 'property')}
    merged += [p for p in find_all(parent, 'property') if p[1] not in own_props]
    subs = []
    for sub in find_all(parent, 'symbol'):
        sub = copy.deepcopy(sub)
        sub[1] = Q(name + sub[1][len(parent[1]):])
        subs.append(sub)
    items = merged + own + subs
    items.sort(key=lambda x: ORDER.index(x[0]) if x[0] in ORDER else 5)
    return ['symbol', Q(name), *items]


def symbol_pins(sym):
    """{unidad: [(número, nombre, tipo, x, y, ángulo)]}; la unidad 0 es común a todas."""
    units = {}
    name = sym[1]
    for sub in find_all(sym, 'symbol'):
        unit, style = sub[1][len(name) + 1:].split('_')
        if style == '2':
            continue
        for pin in find_all(sub, 'pin'):
            at = find(pin, 'at')
            units.setdefault(int(unit), []).append(
                (str(find(pin, 'number')[1]), str(find(pin, 'name')[1]), pin[1],
                 float(at[1]), float(at[2]), float(at[3])))
    return units


# ------------------------------------------------------------------------------------------
# Componentes
# ------------------------------------------------------------------------------------------
class Part:
    """Una unidad de un componente colocada en el esquemático."""

    def __init__(self, ref, lib_id, value, nets, footprint=None, unit=1, dnp=False):
        self.ref, self.lib_id, self.value, self.nets = ref, lib_id, value, nets
        self.footprint, self.unit, self.dnp = footprint, unit, dnp


def two(ref, kind, value, net1, net2, footprint, dnp=False):
    """Pasivo de 2 pines (pin 1 arriba, pin 2 abajo)."""
    lib = {'R': 'Device:R', 'C': 'Device:C', 'CP': 'Device:C_Polarized', 'L': 'Device:L',
           'F': 'Device:Fuse'}[kind]
    return Part(ref, lib, value, {'1': net1, '2': net2}, footprint, dnp=dnp)


def build_parts(cfg):
    """Lista de bloques: (título, [Part]). Todos los valores salen del YAML."""
    an, bd, bat = cfg['analog'], cfg['board'], cfg['battery']['divider']
    gpio_net = {p['gpio']: p['func'] for p in cfg['pins'] if not p.get('onboard')}

    devkit = {}
    for i, pname in enumerate(DEVKIT_LEFT + DEVKIT_RIGHT):
        pnum = str(i + 1)
        if pname == 'GND':
            devkit[pnum] = 'GND'
        elif pname == '3V3':
            devkit[pnum] = '+3V3'
        elif pname.startswith('IO') and int(pname[2:].split('_')[0]) in gpio_net:
            devkit[pnum] = gpio_net[int(pname[2:].split('_')[0])]
        else:
            devkit[pnum] = None   # sin conexión (EN, VIN, UART0, GPIO libres, LED de la placa)

    power = [
        Part('J1', 'Connector_Generic:Conn_01x02', 'BAT 2S (salida BMS)', {'1': 'VBAT_RAW', '2': 'GND'}, FP['XT30_M']),
        Part('SW1', 'Switch:SW_SPST', 'Interruptor 5 A', {'1': 'VBAT_RAW', '2': 'VBAT_SW'}, FP['SW']),
        two('F1', 'F', f'{cfg["battery"]["fuse_a"]:g}A', 'VBAT_SW', 'VBAT_F', FP['FUSE']),
        Part('Q1', 'Transistor_FET:IRF4905', 'IRF4905', {'D': 'VBAT_F', 'S': 'VBAT', 'G': 'Q1_G'}, FP['TO220']),
        two('R1', 'R', si(bd['pfet_gate_r']), 'Q1_G', 'GND', FP['R']),
        two('C1', 'CP', si(num(bd['c_bulk_vbat'])) + 'F/16V', 'VBAT', 'GND', FP['CP_big']),
        Part('U1', f'{PROJECT}:Buck_Module', 'MP1584 5V 3A (5V_USB)',
             {'1': 'VBAT', '2': 'GND', '3': '5V_USB', '4': 'GND'}, FP['HDR4']),
        two('C2', 'CP', si(num(bd['c_bulk_5v'])) + 'F/10V', '5V_USB', 'GND', FP['CP']),
        Part('J2', 'Connector_Generic:Conn_01x02', 'Hub USB (5V_USB)', {'1': '5V_USB', '2': 'GND'}, FP['XT30_F']),
        Part('U2', f'{PROJECT}:Buck_Module', 'MP1584 5V (5V_AUX)',
             {'1': 'VBAT', '2': 'GND', '3': '5V_AUX_RAW', '4': 'GND'}, FP['HDR4']),
        two('L1', 'L', si(num(bd['l_aux'])) + 'H', '5V_AUX_RAW', '5V_AUX', FP['L']),
        two('C3', 'CP', si(num(bd['c_bulk_5v'])) + 'F/10V', '5V_AUX', 'GND', FP['CP']),
        two('C4', 'C', si(num(bd['c_decoupling'])), '5V_AUX', 'GND', FP['C']),
        two('R2', 'R', si(bd['led_r']), '5V_AUX', 'LED_PWR', FP['R']),
        Part('D1', 'Device:LED', 'LED encendido', {'1': 'GND', '2': 'LED_PWR'}, FP['LED']),
        Part('#FLG01', 'power:PWR_FLAG', 'PWR_FLAG', {'1': 'GND'}),
        Part('#FLG02', 'power:PWR_FLAG', 'PWR_FLAG', {'1': 'VBAT'}),
    ]

    enc_pu, enc_pd = si(an['encoder']['pullup']), si(an['encoder']['alt_r_bottom'])
    mcu = [
        Part('U3', f'{PROJECT}:ESP32_DevKitV1_30', 'ESP32 DevKit V1', devkit),
        two('C5', 'C', si(num(bd['c_decoupling'])), '+3V3', 'GND', FP['C']),
        Part('U4', f'{PROJECT}:TB6612_Module', 'TB6612FNG módulo',
             {'1': 'GND', '2': '+3V3', '3': 'MOT_L_A', '4': 'MOT_L_B', '5': 'MOT_R_B', '6': 'MOT_R_A',
              '7': 'VBAT', '8': 'GND', '9': 'MOT_L_PWM', '10': 'MOT_L_IN2', '11': 'MOT_L_IN1',
              '12': 'MOT_STBY', '13': 'MOT_R_IN1', '14': 'MOT_R_IN2', '15': 'MOT_R_PWM', '16': 'GND'},
             FP['POLOLU16']),
        two('C6', 'CP', si(num(bd['c_bulk_5v'])) + 'F/16V', 'VBAT', 'GND', FP['CP']),
        two('R3', 'R', si(bd['stby_pulldown']), 'MOT_STBY', 'GND', FP['R']),
        Part('J3', 'Connector_Generic:Conn_01x06', 'Motor izq. + encoder',
             {'1': 'MOT_L_A', '2': 'MOT_L_B', '3': '5V_AUX', '4': 'GND', '5': 'ENC_L_A', '6': 'ENC_L_B'}, FP['XH6']),
        Part('J4', 'Connector_Generic:Conn_01x06', 'Motor der. + encoder',
             {'1': 'MOT_R_A', '2': 'MOT_R_B', '3': '5V_AUX', '4': 'GND', '5': 'ENC_R_A', '6': 'ENC_R_B'}, FP['XH6']),
    ]
    for i, net in enumerate(['ENC_L_A', 'ENC_L_B', 'ENC_R_A', 'ENC_R_B']):
        mcu.append(two(f'R{4 + i}', 'R', enc_pu, '+3V3', net, FP['R']))
    for i, net in enumerate(['ENC_L_A', 'ENC_L_B', 'ENC_R_A', 'ENC_R_B']):
        mcu.append(two(f'R{8 + i}', 'R', enc_pd, net, 'GND', FP['R'], dnp=True))
    mcu += [
        Part('U5', f'{PROJECT}:GY521_Module', 'GY-521 (MPU6050)',
             {'1': '+3V3', '2': 'GND', '3': 'I2C_SCL', '4': 'I2C_SDA', '5': None, '6': None,
              '7': 'GND', '8': 'IMU_INT'}, FP['SOCK8']),
        two('R12', 'R', si(bd['i2c_pullup']), '+3V3', 'I2C_SDA', FP['R']),
        two('R13', 'R', si(bd['i2c_pullup']), '+3V3', 'I2C_SCL', FP['R']),
        Part('J5', 'Connector_Generic:Conn_01x04', 'MLX90640 (fase 2)',
             {'1': '+3V3', '2': 'GND', '3': 'I2C_SCL', '4': 'I2C_SDA'}, FP['SOCK4'], dnp=True),
    ]

    co2, rad, vr, ht = an['co2'], an['radar'], an['vref'], an['heater']
    opamp = 'Amplifier_Operational:MCP6004'
    analog = [
        Part('U6', opamp, 'MCP6004', {'3': 'CO2_EMF', '2': 'CO2_FB', '1': 'CO2_OUT'}, FP['DIP14'], unit=1),
        Part('U6', opamp, 'MCP6004', {'5': 'VREF', '6': 'RADAR_INV1', '7': 'RADAR_O1'}, FP['DIP14'], unit=2),
        Part('U6', opamp, 'MCP6004', {'10': 'VREF', '9': 'RADAR_INV2', '8': 'RADAR_O2'}, FP['DIP14'], unit=3),
        Part('U6', opamp, 'MCP6004', {'12': 'VREF_DIV', '13': 'VREF', '14': 'VREF'}, FP['DIP14'], unit=4),
        Part('U6', opamp, 'MCP6004', {'4': '+3V3', '11': 'GND'}, FP['DIP14'], unit=5),
        two('C7', 'C', si(num(bd['c_decoupling'])), '+3V3', 'GND', FP['C']),
        Part('J6', f'{PROJECT}:TGS4161', 'TGS4161 (CO2)',
             {'1': '5V_AUX', '2': 'HEATER_SW', '3': 'CO2_EMF', '4': 'GND'}, FP['XH4']),
        two('R14', 'R', si(co2['gain_rf']), 'CO2_OUT', 'CO2_FB', FP['R']),
        two('R15', 'R', si(co2['gain_rg']), 'CO2_FB', 'GND', FP['R']),
        two('R16', 'R', si(co2['out_r']), 'CO2_OUT', 'ADC_CO2', FP['R']),
        two('C8', 'C', si(num(co2['out_c'])), 'ADC_CO2', 'GND', FP['C']),
        Part('Q2', 'Transistor_FET:AO3400A', 'AO3400A', {'G': 'HEATER_G', 'S': 'GND', 'D': 'HEATER_SW'}),
        two('R17', 'R', si(ht['r_gate']), 'CO2_HEATER_EN', 'HEATER_G', FP['R']),
        two('R18', 'R', si(ht['r_gate_pulldown']), 'HEATER_G', 'GND', FP['R']),
        two('R19', 'R', si(vr['r_top']), '+3V3', 'VREF_DIV', FP['R']),
        two('R20', 'R', si(vr['r_bottom']), 'VREF_DIV', 'GND', FP['R']),
        two('C9', 'C', si(num(vr['c'])), 'VREF_DIV', 'GND', FP['C_SMD']),
        Part('J7', 'Connector_Generic:Conn_01x03', 'HB100 (radar)',
             {'1': '5V_AUX', '2': 'RADAR_IF', '3': 'GND'}, FP['XH3']),
        two('C10', 'C', si(num(rad['cin'])), 'RADAR_IF', 'RADAR_N1', FP['C_SMD']),
        two('R21', 'R', si(rad['rin']), 'RADAR_N1', 'RADAR_INV1', FP['R']),
        two('R22', 'R', si(rad['rf']), 'RADAR_O1', 'RADAR_INV1', FP['R']),
        two('C11', 'C', si(num(rad['cf'])), 'RADAR_O1', 'RADAR_INV1', FP['C']),
        two('C12', 'C', si(num(rad['cin'])), 'RADAR_O1', 'RADAR_N2', FP['C_SMD']),
        two('R23', 'R', si(rad['rin']), 'RADAR_N2', 'RADAR_INV2', FP['R']),
        two('R24', 'R', si(rad['rf']), 'RADAR_O2', 'RADAR_INV2', FP['R']),
        two('C13', 'C', si(num(rad['cf'])), 'RADAR_O2', 'RADAR_INV2', FP['C']),
        two('R25', 'R', si(rad['out_r']), 'RADAR_O2', 'ADC_RADAR', FP['R']),
        two('C14', 'C', si(num(rad['out_c'])), 'ADC_RADAR', 'GND', FP['C_SMD']),
        two('R26', 'R', si(bat['r_top']), 'VBAT', 'ADC_VBAT', FP['R']),
        two('R27', 'R', si(bat['r_bottom']), 'ADC_VBAT', 'GND', FP['R']),
        two('C15', 'C', si(num(bat['c_filter'])), 'ADC_VBAT', 'GND', FP['C']),
    ]
    return [
        ('ALIMENTACIÓN — batería 2S, protección, reguladores 5V_USB y 5V_AUX', power, (20, 30), 270),
        ('ESP32, MOTORES, ENCODERS E IMU', mcu, (20, 150), 270),
        ('ANALÓGICO — CO2 (TGS4161), RADAR (HB100), VREF y BATERÍA', analog, (310, 30), 270),
    ]


# ------------------------------------------------------------------------------------------
# Colocación y escritura del esquemático
# ------------------------------------------------------------------------------------------
def snap(v):
    return math.ceil(v / GRID) * GRID


def pin_geometry(pin):
    """Punto de conexión (coordenadas del esquemático, y hacia abajo) y dirección hacia afuera."""
    _, _, _, x, y, angle = pin
    rad = math.radians(angle)
    out = (-round(math.cos(rad)), round(math.sin(rad)))
    return (x, -y), out


def layout_item(part, pins):
    """Caja envolvente relativa al origen del símbolo, incluyendo etiquetas."""
    xs, ys = [0.0], [0.0]
    for pin in pins:
        (px, py), (ox, oy) = pin_geometry(pin)
        net = part.nets.get(pin[0], part.nets.get(pin[1]))
        length = GRID + (len(net) * 1.05 + 1.5 if net else 0)
        xs += [px, px + ox * length]
        ys += [py, py + oy * length]
        if ox:
            ys += [py - 1.5, py + 1.5]
        else:
            xs += [px - 1.5, px + 1.5]
    return min(xs) - 1.27, min(ys) - 1.27, max(xs) + 1.27, max(ys) + 1.27


def generate():
    cfg = yaml.safe_load(HW_YAML.read_text(encoding='utf-8'))
    custom = custom_symbols()
    lib = ['kicad_symbol_lib', ['version', 20251024], ['generator', Q('gen_schematic')],
           ['generator_version', Q('10.0')], *custom]
    (KICAD_DIR / f'{PROJECT}.kicad_sym').write_text(dump(lib) + '\n', encoding='utf-8')
    _LIB_CACHE.pop(PROJECT, None)

    root = uid('root')
    blocks = build_parts(cfg)
    lib_symbols, items = {}, []
    net_count = {}
    flags = 0

    for b, (title, parts, (x0, y0), width) in enumerate(blocks):
        items.append(['text', Q(title), ['exclude_from_sim', 'no'], ['at', x0, y0, 0],
                      ['effects', ['font', ['size', 2.54, 2.54], ['thickness', 0.508], ['bold', 'yes']],
                       ['justify', 'left', 'bottom']], ['uuid', uid('text', b)]])
        cx, cy, row_h = x0, y0 + 8, 0.0
        for part in parts:
            lib_name, sym_name = part.lib_id.split(':')
            if part.lib_id not in lib_symbols:
                sym = flat_symbol(lib_name, sym_name)
                sym[1] = Q(part.lib_id)
                lib_symbols[part.lib_id] = sym
            sym = lib_symbols[part.lib_id]
            all_pins = symbol_pins(['symbol', Q(sym_name), *sym[2:]])
            pins = all_pins.get(part.unit, []) + all_pins.get(0, [])
            for pin in pins:
                if pin[0] not in part.nets and pin[1] not in part.nets:
                    raise SystemExit(f'{part.ref}: pin {pin[0]} ({pin[1]}) sin red ni "None" explícito')

            bx0, by0, bx1, by1 = layout_item(part, pins)
            text_w = max(len(part.ref), len(part.value)) * 1.27 + 4
            w, h = bx1 - bx0 + text_w, by1 - by0
            if cx + w > x0 + width and cx > x0:
                cx, cy, row_h = x0, cy + row_h + 5, 0.0
            X, Y = snap(cx - bx0), snap(cy - by0)
            cx += w + 4
            row_h = max(row_h, h)

            key = (part.ref, part.unit)
            is_flag = part.ref.startswith('#')
            props = [
                ['property', Q('Reference'), Q(part.ref), ['at', X + bx1 + 1.27, Y + by0 + 2.54, 0],
                 ['effects', ['font', ['size', 1.27, 1.27]], ['justify', 'left']] + ([['hide', 'yes']] if is_flag else [])],
                ['property', Q('Value'), Q(part.value), ['at', X + bx1 + 1.27, Y + by0 + 5.08, 0],
                 ['effects', ['font', ['size', 1.27, 1.27]], ['justify', 'left']]],
            ]
            fp = part.footprint
            if fp is None:
                fp_node = find_all(sym, 'property')
                fp = next((str(p[2]) for p in fp_node if p[1] == 'Footprint'), '')
            props += [['property', Q('Footprint'), Q(fp), ['at', X, Y, 0], hidden()],
                      ['property', Q('Datasheet'), Q(''), ['at', X, Y, 0], hidden()],
                      ['property', Q('Description'), Q(''), ['at', X, Y, 0], hidden()]]
            inst = ['symbol', ['lib_id', Q(part.lib_id)], ['at', X, Y, 0], ['unit', part.unit],
                    ['exclude_from_sim', 'no'], ['in_bom', 'no' if is_flag else 'yes'], ['on_board', 'yes'],
                    ['dnp', 'yes' if part.dnp else 'no'], ['uuid', uid('sym', *key)], *props]
            for pin in pins:
                inst.append(['pin', Q(pin[0]), ['uuid', uid('pin', *key, pin[0])]])
            inst.append(['instances', ['project', Q(PROJECT),
                                       ['path', Q('/' + root), ['reference', Q(part.ref)], ['unit', part.unit]]]])
            items.append(inst)

            for pin in pins:
                (px, py), (ox, oy) = pin_geometry(pin)
                ax, ay = X + px, Y + py
                net = part.nets.get(pin[0], part.nets.get(pin[1]))
                tag = (*key, pin[0])
                if net is None:
                    items.append(['no_connect', ['at', ax, ay], ['uuid', uid('nc', *tag)]])
                    continue
                net_count[net] = net_count.get(net, 0) + 1
                ex, ey = ax + ox * GRID, ay + oy * GRID
                items.append(['wire', ['pts', ['xy', ax, ay], ['xy', ex, ey]],
                              ['stroke', ['width', 0], ['type', 'default']], ['uuid', uid('wire', *tag)]])
                angle = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(ox, oy)]
                just = 'left' if angle in (0, 90) else 'right'
                items.append(['label', Q(net), ['at', ex, ey, angle], ['fields_autoplaced', 'yes'],
                              ['effects', ['font', ['size', 1.27, 1.27]], ['justify', just, 'bottom']],
                              ['uuid', uid('label', *tag)]])
            flags += is_flag

    lonely = sorted(n for n, c in net_count.items() if c < 2)
    if lonely:
        raise SystemExit(f'Redes con un solo pin (revisar): {lonely}')

    sch = ['kicad_sch', ['version', 20250114], ['generator', Q('eeschema')], ['generator_version', Q('9.0')],
           ['uuid', root], ['paper', Q('A2')],
           ['title_block', ['title', Q('Findbot — placa base (módulos)')], ['rev', Q('H3')],
            ['company', Q('UNAD — Ingeniería Electrónica')],
            ['comment', 1, Q('Generado por hardware/kicad/gen_schematic.py desde config/robot_hw.yaml — no editar a mano')]],
           ['lib_symbols', *lib_symbols.values()], *items,
           ['sheet_instances', ['path', Q('/'), ['page', Q('1')]]], ['embedded_fonts', 'no']]
    (KICAD_DIR / f'{PROJECT}.kicad_sch').write_text(dump(sch) + '\n', encoding='utf-8')
    (KICAD_DIR / 'sym-lib-table').write_text(
        '(sym_lib_table\n\t(version 7)\n\t(lib (name "findbot") (type "KiCad") '
        '(uri "${KIPRJMOD}/findbot.kicad_sym") (options "") (descr "Módulos de la placa base"))\n)\n',
        encoding='utf-8')
    pro = KICAD_DIR / f'{PROJECT}.kicad_pro'
    if not pro.exists():
        pro.write_text('{\n  "meta": {\n    "filename": "findbot.kicad_pro",\n    "version": 3\n  }\n}\n',
                       encoding='utf-8')
    n_parts = len({p.ref for _, parts, _, _ in blocks for p in parts if not p.ref.startswith('#')})
    print(f'{n_parts} componentes, {len(net_count)} redes → {PROJECT}.kicad_sch')


if __name__ == '__main__':
    generate()
