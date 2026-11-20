#!/usr/bin/env python3
"""Genera include/config_gen.h desde hardware/config/robot_hw.yaml.

Todo valor numérico del firmware (pines, geometría, límites, umbrales, ganancias) sale de aquí:
el código C++ no tiene números "quemados".
Uso:
  python3 tools/gen_config.py           # escribe el encabezado
  python3 tools/gen_config.py --check   # sale con 1 si el encabezado o platformio.ini no coinciden
"""
import configparser
import math
import sys
from pathlib import Path

import yaml

FW_DIR = Path(__file__).resolve().parents[1]
HW_YAML = FW_DIR.parent / 'config' / 'robot_hw.yaml'
HEADER = FW_DIR / 'include' / 'config_gen.h'


def f(value):
    """Literal float de C++."""
    return f'{float(value)!r}f'


def render(cfg):
    """Devuelve el texto del encabezado."""
    geo, mot, bat, an, fw = cfg['geometry'], cfg['motors'], cfg['battery'], cfg['analog'], cfg['firmware']
    counts = int(mot['encoder_ppr_motor']) * int(mot['gear_ratio']) * 4
    if counts != int(mot['counts_per_wheel_rev']):
        raise SystemExit(f'counts_per_wheel_rev={mot["counts_per_wheel_rev"]} no es ppr×reducción×4={counts}')
    div = bat['divider']
    co2 = an['co2']
    vref = an['vref']
    lines = [
        '// Generado por tools/gen_config.py desde hardware/config/robot_hw.yaml — NO EDITAR.',
        '#pragma once',
        '#include <stdint.h>',
        '',
        'namespace cfg {',
        '',
        '// --- Pines (robot_hw.yaml → pins) ---',
    ]
    for pin in cfg['pins']:
        lines.append(f'constexpr int PIN_{pin["func"]} = {int(pin["gpio"])};')
    lines += [
        f'constexpr uint32_t USB_BAUD = {int(cfg["mcu"]["usb_serial_baud"])};',
        '',
        '// --- Geometría y límites (geometry) ---',
        f'constexpr float WHEEL_RADIUS = {f(geo["wheels"]["radius"])};',
        f'constexpr float WHEEL_SEPARATION = {f(geo["wheels"]["separation"])};',
        f'constexpr float MAX_LIN_VEL = {f(geo["limits"]["max_linear_velocity"])};',
        f'constexpr float MAX_ANG_VEL = {f(geo["limits"]["max_angular_velocity"])};',
        f'constexpr float MAX_LIN_ACC = {f(geo["limits"]["max_linear_acceleration"])};',
        f'constexpr float MAX_ANG_ACC = {f(geo["limits"]["max_angular_acceleration"])};',
        '',
        '// --- Motores y encoders (motors) ---',
        f'constexpr int32_t COUNTS_PER_WHEEL_REV = {counts};',
        f'constexpr float MOTOR_V_RATED = {f(mot["v_rated"])};',
        f'constexpr float WHEEL_MAX_RAD_S = {f(mot["no_load_rpm_at_6v"] * 2 * math.pi / 60)};',
        f'constexpr uint32_t MOTOR_PWM_FREQ = {int(mot["pwm_frequency"])};',
        'constexpr uint8_t MOTOR_PWM_BITS = 10;',
        f'constexpr float MOTOR_DEADBAND = {f(fw["motor_deadband"])};',
        f'constexpr bool ENCODER_USE_PCNT = {"true" if fw["encoder_backend"] == "pcnt" else "false"};',
        f'constexpr uint16_t ENCODER_GLITCH_FILTER = {int(fw["encoder_glitch_filter_ticks"])};',
        f'constexpr bool MOTOR_INVERT_LEFT = {str(bool(fw["motor_invert_left"])).lower()};',
        f'constexpr bool MOTOR_INVERT_RIGHT = {str(bool(fw["motor_invert_right"])).lower()};',
        f'constexpr bool ENCODER_INVERT_LEFT = {str(bool(fw["encoder_invert_left"])).lower()};',
        f'constexpr bool ENCODER_INVERT_RIGHT = {str(bool(fw["encoder_invert_right"])).lower()};',
        '',
        '// --- Batería (battery) ---',
        f'constexpr float BAT_DIVIDER_RATIO = {f(div["r_bottom"] / (div["r_top"] + div["r_bottom"]))};',
        f'constexpr float BAT_V_FULL = {f(bat["v_full"])};',
        f'constexpr float BAT_V_LOW = {f(bat["v_low"])};',
        f'constexpr float BAT_V_CRITICAL = {f(bat["v_critical"])};',
        f'constexpr float BAT_V_CUTOFF = {f(bat["v_cutoff_bms"])};',
        f'constexpr float BAT_FILTER_ALPHA = {f(fw["battery_filter_alpha"])};',
        f'constexpr float BAT_HYSTERESIS = {f(fw["battery_hysteresis_v"])};',
        f'constexpr int BAT_ADC_ATTEN_DB = {int(div["adc_attenuation_db"])};',
        '',
        '// --- Sensores analógicos (analog) ---',
        f'constexpr float CO2_GAIN = {f(1 + co2["gain_rf"] / co2["gain_rg"])};',
        f'constexpr float CO2_SLOPE_V_PER_DECADE = {f(fw["co2"]["slope_v_per_decade"])};',
        f'constexpr float CO2_REF_PPM = {f(fw["co2"]["ref_ppm"])};',
        f'constexpr float RADAR_VREF = {f(an["esp32_io"]["vdd"] * vref["r_bottom"] / (vref["r_top"] + vref["r_bottom"]))};',
        f'constexpr int RADAR_RMS_WINDOW = {int(fw["radar_rms_window"])};',
        f'constexpr int CO2_ADC_ATTEN_DB = {int(co2["adc_attenuation_db"])};',
        f'constexpr int RADAR_ADC_ATTEN_DB = {int(an["radar"]["adc_attenuation_db"])};',
        '',
        '// --- IMU MPU6050 ---',
        f'constexpr int IMU_ACCEL_RANGE_G = {int(fw["imu"]["accel_range_g"])};',
        f'constexpr int IMU_GYRO_RANGE_DPS = {int(fw["imu"]["gyro_range_dps"])};',
        f'constexpr uint8_t IMU_DLPF_CFG = {int(fw["imu"]["dlpf_cfg"])};',
        f'constexpr int IMU_CALIB_SAMPLES = {int(fw["imu"]["calib_samples"])};',
        f'constexpr uint8_t IMU_I2C_ADDRESS = 0x{int(fw["imu"]["i2c_address"]):02X};',
        f'constexpr uint32_t IMU_I2C_HZ = {int(fw["imu"]["i2c_hz"])};',
        '',
        '// --- Control y tiempos (firmware) ---',
    ]
    for name, hz in fw['rates_hz'].items():
        lines.append(f'constexpr uint32_t PERIOD_{name.upper()}_US = {int(round(1e6 / hz))};')
    pid = fw['pid']
    lines += [
        f'constexpr uint32_t CMD_TIMEOUT_MS = {int(round(fw["cmd_timeout_s"] * 1000))};',
        f'constexpr float PID_KP = {f(pid["kp"])};',
        f'constexpr float PID_KI = {f(pid["ki"])};',
        f'constexpr float PID_KD = {f(pid["kd"])};',
        f'constexpr float PID_I_LIMIT = {f(pid["i_limit"])};',
        f'constexpr float PID_OUT_LIMIT = {f(pid["out_limit"])};',
        '',
        '// --- micro-ROS (firmware.ros) ---',
    ]
    ros = fw['ros']

    def s(text):
        return '"' + str(text).replace('\\', '\\\\').replace('"', '\\"') + '"'

    def farr(values):
        return '{' + ', '.join(f(v) for v in values) + '}'

    if len(ros['joints']) != 2:
        raise SystemExit('firmware.ros.joints debe tener exactamente 2 nombres (izquierda, derecha)')
    for key in ('odom_pose', 'odom_twist'):
        if len(ros['covariance'][key]) != 6:
            raise SystemExit(f'firmware.ros.covariance.{key} debe tener 6 valores')
    lines.append(f'constexpr const char* ROS_NODE_NAME = {s(ros["node_name"])};')
    lines += [
        '// FINDBOT_ROS_DOMAIN_E2E lo define solo el entorno esp32_e2e (prueba aislada)',
        '#ifdef FINDBOT_ROS_DOMAIN_E2E',
        f'constexpr uint32_t ROS_DOMAIN_ID = {int(ros["e2e"]["domain_id"])};',
        f'constexpr uint32_t ENTITY_TIMEOUT_MS = {int(ros["e2e"]["entity_timeout_ms"])};',
        f'constexpr int AGENT_TIMEOUT_MS = {int(ros["e2e"]["agent_timeout_ms"])};',
        '#else',
        f'constexpr uint32_t ROS_DOMAIN_ID = {int(ros["domain_id"])};',
        f'constexpr uint32_t ENTITY_TIMEOUT_MS = {int(ros["entity_timeout_ms"])};',
        f'constexpr int AGENT_TIMEOUT_MS = {int(ros["agent_timeout_ms"])};',
        '#endif',
    ]
    for name, topic in ros['topics'].items():
        lines.append(f'constexpr const char* TOPIC_{name.upper()} = {s(topic)};')
    for name, frame in ros['frames'].items():
        lines.append(f'constexpr const char* FRAME_{name.upper()} = {s(frame)};')
    lines += [
        f'constexpr const char* JOINT_LEFT = {s(ros["joints"][0])};',
        f'constexpr const char* JOINT_RIGHT = {s(ros["joints"][1])};',
    ]
    for name, hz in ros['publish_hz'].items():
        lines.append(f'constexpr uint32_t PUB_PERIOD_{name.upper()}_MS = {int(round(1000 / hz))};')
    cov = ros['covariance']
    lines += [
        f'constexpr bool PUBLISH_ODOM_TF = {str(bool(ros["publish_odom_tf"])).lower()};',
        f'constexpr uint32_t AGENT_PING_MS = {int(ros["agent_ping_ms"])};',
        f'constexpr uint32_t AGENT_CHECK_MS = {int(ros["agent_check_ms"])};',
        f'constexpr uint8_t AGENT_ATTEMPTS = {int(ros["agent_attempts"])};',
        f'constexpr int ROS_TASK_CORE = {int(ros["task_core"])};',
        f'constexpr uint32_t ROS_TASK_STACK = {int(ros["task_stack_bytes"])};',
        f'constexpr float COV_ODOM_POSE[6] = {farr(cov["odom_pose"])};',
        f'constexpr float COV_ODOM_TWIST[6] = {farr(cov["odom_twist"])};',
        f'constexpr float COV_IMU_GYRO = {f(cov["imu_gyro"])};',
        f'constexpr float COV_IMU_ACCEL = {f(cov["imu_accel"])};',
        f'constexpr float BAT_DESIGN_CAPACITY_AH = {f(bat["capacity_ah"])};',
        '',
        '}  // namespace cfg',
        '',
    ]
    return '\n'.join(lines)


def check_ini(cfg):
    """platformio.ini no puede leer el YAML: se verifica que monitor_speed coincida."""
    ini = configparser.ConfigParser()
    ini.read(FW_DIR / 'platformio.ini')
    speed = int(ini['esp32_base']['monitor_speed'])
    baud = int(cfg['mcu']['usb_serial_baud'])
    return [] if speed == baud else [f'platformio.ini monitor_speed={speed} ≠ usb_serial_baud={baud}']


def main(argv):
    cfg = yaml.safe_load(HW_YAML.read_text(encoding='utf-8'))
    text = render(cfg)
    if '--check' in argv:
        errors = check_ini(cfg)
        if not HEADER.exists() or HEADER.read_text(encoding='utf-8') != text:
            errors.append('include/config_gen.h está desactualizado: correr tools/gen_config.py')
        for err in errors:
            print(f'ERROR  {err}')
        print('config_gen.h y platformio.ini coinciden con robot_hw.yaml' if not errors else '')
        return 1 if errors else 0
    if not HEADER.exists() or HEADER.read_text(encoding='utf-8') != text:
        HEADER.write_text(text, encoding='utf-8')
        print('config_gen.h regenerado desde robot_hw.yaml')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
