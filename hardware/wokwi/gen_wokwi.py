#!/usr/bin/env python3
"""Genera la simulación Wokwi del firmware (Fase H5) desde hardware/config/robot_hw.yaml.

Salida en hardware/wokwi/:
  diagram.json       circuito: ESP32 DevKit V1, MPU6050, 2 encoders KY-040, 3 potenciómetros
                     (batería, CO2, radar), analizador lógico en motores y LED del calefactor
  scenario_h5.yaml   prueba automática para wokwi-cli (mandos por serial + niveles de pines)
  wokwi.toml         apunta al firmware compilado por PlatformIO
Los pines, sentidos de giro y umbrales salen del YAML: el circuito y la prueba no pueden
quedar distintos del firmware. Uso: python3 hardware/wokwi/gen_wokwi.py
"""
import json
from pathlib import Path

import yaml

WOKWI_DIR = Path(__file__).resolve().parent
HW_YAML = WOKWI_DIR.parent / 'config' / 'robot_hw.yaml'
VDD = 3.3
POT_MAX = 1023   # rango del atributo "value" del potenciómetro de Wokwi

# Rótulos de la DevKit V1 de Wokwi para los GPIO que no se llaman "D<n>"
SPECIAL_LABELS = {36: 'VP', 39: 'VN', 1: 'TX0', 3: 'RX0', 16: 'RX2', 17: 'TX2'}


def label(gpio):
    return SPECIAL_LABELS.get(gpio, f'D{gpio}')


def build(cfg, ci=False):
    """Circuito y escenario. ci=True cambia las perillas KY-040 (sin control automatizable)
    por 2 botones sin rebote por encoder, que el escenario presiona en orden de cuadratura."""
    pins = {p['func']: p['gpio'] for p in cfg['pins']}
    pin = {f: f'esp:{label(g)}' for f, g in pins.items()}
    bat = cfg['battery']
    div = bat['divider']
    ratio = div['r_bottom'] / (div['r_top'] + div['r_bottom'])
    an = cfg['analog']
    co2_gain = 1 + an['co2']['gain_rf'] / an['co2']['gain_rg']
    vref = an['esp32_io']['vdd'] * an['vref']['r_bottom'] / (an['vref']['r_top'] + an['vref']['r_bottom'])
    emf_clean_air = (an['co2']['emf_min'] + an['co2']['emf_max']) / 2

    def pot_value(volts):
        return str(round(min(max(volts / VDD, 0.0), 1.0) * POT_MAX))

    parts = [
        {'type': 'wokwi-esp32-devkit-v1', 'id': 'esp', 'top': 0, 'left': 0, 'attrs': {}},
        {'type': 'wokwi-mpu6050', 'id': 'imu', 'top': -150, 'left': 250, 'attrs': {}},
        {'type': 'wokwi-potentiometer', 'id': 'pot_bat', 'top': -150, 'left': -250,
         'attrs': {'value': pot_value(bat['v_nominal'] * ratio)}},
        {'type': 'wokwi-potentiometer', 'id': 'pot_co2', 'top': -150, 'left': -130,
         'attrs': {'value': pot_value(emf_clean_air * co2_gain)}},
        {'type': 'wokwi-potentiometer', 'id': 'pot_radar', 'top': -150, 'left': -10,
         'attrs': {'value': pot_value(vref)}},
        {'type': 'wokwi-logic-analyzer', 'id': 'la', 'top': 250, 'left': 250,
         'attrs': {'bufferSize': '1000000', 'channels': '8'}},
        {'type': 'wokwi-led', 'id': 'led_heater', 'top': 120, 'left': 250, 'attrs': {'color': 'orange'}},
        {'type': 'wokwi-resistor', 'id': 'r_heater', 'top': 170, 'left': 200, 'attrs': {'value': '220'}},
    ]
    gnd, v33 = 'esp:GND.1', 'esp:3V3'
    w = []

    def wire(a, b, color):
        w.append([a, b, color, []])

    # IMU por I2C (AD0 a GND → 0x68, igual que la placa)
    wire('imu:VCC', v33, 'red')
    wire('imu:GND', gnd, 'black')
    wire('imu:SDA', pin['I2C_SDA'], 'green')
    wire('imu:SCL', pin['I2C_SCL'], 'blue')
    wire('imu:INT', pin['IMU_INT'], 'violet')
    wire('imu:AD0', gnd, 'black')
    for row, side in enumerate(('L', 'R')):
        top = 250 + 100 * row
        if ci:
            # Botón presionado = canal en 0 (contra GND); suelto = 1 por el pull-up del ESP32
            for col, ch in enumerate(('A', 'B')):
                bid = f'btn_{side.lower()}_{ch.lower()}'
                parts.append({'type': 'wokwi-pushbutton', 'id': bid, 'top': top, 'left': -250 + 90 * col,
                              'attrs': {'color': 'yellow' if ch == 'A' else 'orange', 'bounce': '0',
                                        'label': f'ENC_{side}_{ch}'}})
                wire(f'{bid}:1.l', pin[f'ENC_{side}_{ch}'], 'yellow' if ch == 'A' else 'orange')
                wire(f'{bid}:2.l', gnd, 'black')
        else:
            # Encoders: CLK = canal A, DT = canal B (cuadratura real al girar la perilla)
            enc = f'enc_{side.lower()}'
            parts.append({'type': 'wokwi-ky-040', 'id': enc, 'top': top, 'left': -250, 'attrs': {}})
            wire(f'{enc}:VCC', v33, 'red')
            wire(f'{enc}:GND', gnd, 'black')
            wire(f'{enc}:CLK', pin[f'ENC_{side}_A'], 'yellow')
            wire(f'{enc}:DT', pin[f'ENC_{side}_B'], 'orange')
    # Potenciómetros: su salida es directamente la tensión en el pin del ADC
    for pot, func in (('pot_bat', 'ADC_VBAT'), ('pot_co2', 'ADC_CO2'), ('pot_radar', 'ADC_RADAR')):
        wire(f'{pot}:VCC', v33, 'red')
        wire(f'{pot}:GND', gnd, 'black')
        wire(f'{pot}:SIG', pin[func], 'cyan')
    # Analizador lógico: lo que vería el TB6612
    for ch, func in enumerate(['MOT_L_PWM', 'MOT_L_IN1', 'MOT_L_IN2', 'MOT_R_PWM',
                               'MOT_R_IN1', 'MOT_R_IN2', 'MOT_STBY', 'CO2_HEATER_EN']):
        wire(f'la:D{ch}', pin[func], 'gray')
    wire('la:GND', gnd, 'black')
    # Calefactor del TGS4161 → LED
    wire(pin['CO2_HEATER_EN'], 'r_heater:1', 'orange')
    wire('r_heater:2', 'led_heater:A', 'orange')
    wire('led_heater:C', 'esp:GND.2', 'black')

    diagram = {'version': 1, 'author': 'findbot (generado por gen_wokwi.py)', 'editor': 'wokwi',
               'parts': parts, 'connections': w, 'dependencies': {}}

    # ---------------- Prueba automática ----------------
    fw = cfg['firmware']
    inv_l, inv_r = fw['motor_invert_left'], fw['motor_invert_right']

    def forward_levels(invert):
        # duty > 0 → IN1 = 1, IN2 = 0; con inversión del YAML, al revés
        return (0, 1) if invert else (1, 0)

    l1, l2 = forward_levels(inv_l)
    r1, r2 = forward_levels(inv_r)
    lbl = {f: label(g) for f, g in pins.items()}
    v_crit_test = bat['v_critical'] - 1.0   # bien por debajo del umbral
    pwm_hz = cfg['motors']['pwm_frequency']
    gyro_test_dps = 90.0
    gz_expected = f'{gyro_test_dps * 3.14159265 / 180.0:.2f}'

    def expect(func, value):
        return {'expect-pin': {'part-id': 'esp', 'name': lbl[func], 'value': value}}

    # Cuadratura simulada con botones: A↓ B↓ A↑ B↑ = 4 flancos = +4 cuentas crudas por ciclo
    # (configuración ×4 del PCNT en encoders.cpp); luego se aplica la inversión del YAML.
    quad_cycles = 5

    def quadrature(side):
        out = []
        for _ in range(quad_cycles):
            for ch, pressed in (('a', 1), ('b', 1), ('a', 0), ('b', 0)):
                out.append({'set-control': {'part-id': f'btn_{side}_{ch}', 'name': 'pressed',
                                            'value': pressed}})
                out.append({'delay': '20ms'})
        return out

    enc_l = 4 * quad_cycles * (-1 if fw['encoder_invert_left'] else 1)
    enc_r = 4 * quad_cycles * (-1 if fw['encoder_invert_right'] else 1)

    steps = [
        {'wait-serial': 'IMU ok'},
        {'wait-serial': '# T,t_ms'},
        # Calefactor del CO2
        {'write-serial': 'h 1\n'},
        {'delay': '100ms'},
        expect('CO2_HEATER_EN', 1),
        # Mando de avance: motores habilitados, PWM a la frecuencia del YAML, sentidos correctos
        {'write-serial': 'v 0.2 0\n'},
        {'wait-serial': f'# motores ON pwm={pwm_hz} Hz'},
        {'delay': '100ms'},
        expect('MOT_STBY', 1),
        expect('MOT_L_IN1', l1), expect('MOT_L_IN2', l2),
        expect('MOT_R_IN1', r1), expect('MOT_R_IN2', r2),
        # Vigilante: sin mandos nuevos, a los cmd_timeout_s los motores vuelven a 0
        {'wait-serial': '# vigilante: sin mando, motores a 0'},
        {'delay': '800ms'},
        expect('MOT_L_IN1', 0), expect('MOT_L_IN2', 0),
        expect('MOT_R_IN1', 0), expect('MOT_R_IN2', 0),
        # IMU: girar el MPU6050 a 90 °/s en z → gz ≈ 1.57 rad/s
        {'set-control': {'part-id': 'imu', 'name': 'rotationZ', 'value': gyro_test_dps}},
        {'delay': '300ms'},
        {'write-serial': '?\n'},
        {'wait-serial': f'imu_gz={gz_expected}'},
        # Encoders: 5 ciclos de cuadratura en cada rueda → cuentas exactas (con su inversión)
        *quadrature('l'),
        *quadrature('r'),
        {'delay': '100ms'},
        {'write-serial': '?\n'},
        {'wait-serial': f'enc_l={enc_l} enc_r={enc_r}'},
        # Batería crítica → STBY en bajo; al recuperarse, aviso
        {'set-control': {'part-id': 'pot_bat', 'name': 'position',
                         'value': round(v_crit_test * ratio / VDD, 3)}},
        {'wait-serial': '# motores OFF: bateria critica'},
        expect('MOT_STBY', 0),
        {'set-control': {'part-id': 'pot_bat', 'name': 'position',
                         'value': round(bat['v_nominal'] * ratio / VDD, 3)}},
        {'wait-serial': '# bateria recuperada'},
    ]
    scenario = {'name': 'findbot H5 — firmware base en Wokwi', 'version': 1,
                'author': 'gen_wokwi.py', 'steps': steps}
    return diagram, scenario


def main():
    cfg = yaml.safe_load(HW_YAML.read_text(encoding='utf-8'))
    diagram, scenario = build(cfg)
    diagram_ci, _ = build(cfg, ci=True)
    for name, data in (('diagram.json', diagram), ('diagram_ci.json', diagram_ci)):
        (WOKWI_DIR / name).write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n',
                                      encoding='utf-8')
    header = ('# Generado por gen_wokwi.py desde hardware/config/robot_hw.yaml — no editar.\n'
              '# Ejecutar: hardware/wokwi/run_wokwi.sh\n')
    (WOKWI_DIR / 'scenario_h5.yaml').write_text(
        header + yaml.safe_dump(scenario, allow_unicode=True, sort_keys=False, width=100),
        encoding='utf-8')
    (WOKWI_DIR / 'wokwi.toml').write_text(
        '# Generado por gen_wokwi.py — firmware compilado por PlatformIO (entorno esp32_debug: interfaz de texto)\n'
        '[wokwi]\nversion = 1\n'
        "firmware = '../firmware/.pio/build/esp32_debug/firmware.bin'\n"
        "elf = '../firmware/.pio/build/esp32_debug/firmware.elf'\n", encoding='utf-8')
    # Variante micro-ROS (H6): mismo circuito, firmware del entorno esp32 (protocolo binario)
    uros = WOKWI_DIR / 'microros'
    uros.mkdir(exist_ok=True)
    (uros / 'diagram.json').write_text(json.dumps(diagram, indent=2, ensure_ascii=False) + '\n',
                                       encoding='utf-8')
    (uros / 'wokwi.toml').write_text(
        '# Generado por gen_wokwi.py — firmware micro-ROS (entorno esp32_e2e, dominio DDS aislado), prueba de punta a punta H6\n'
        '[wokwi]\nversion = 1\n'
        "firmware = '../../firmware/.pio/build/esp32_e2e/firmware.bin'\n"
        "elf = '../../firmware/.pio/build/esp32_e2e/firmware.elf'\n", encoding='utf-8')
    print(f'diagram.json + diagram_ci.json ({len(diagram["parts"])}/{len(diagram_ci["parts"])} piezas, {len(diagram["connections"])} cables), '
          f'scenario_h5.yaml ({len(scenario["steps"])} pasos), wokwi.toml')


if __name__ == '__main__':
    main()
