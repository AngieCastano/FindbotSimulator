// Generado por tools/gen_config.py desde hardware/config/robot_hw.yaml — NO EDITAR.
#pragma once
#include <stdint.h>

namespace cfg {

// --- Pines (robot_hw.yaml → pins) ---
constexpr int PIN_MOT_L_PWM = 26;
constexpr int PIN_MOT_L_IN1 = 27;
constexpr int PIN_MOT_L_IN2 = 14;
constexpr int PIN_MOT_R_PWM = 25;
constexpr int PIN_MOT_R_IN1 = 33;
constexpr int PIN_MOT_R_IN2 = 32;
constexpr int PIN_MOT_STBY = 13;
constexpr int PIN_ENC_L_A = 18;
constexpr int PIN_ENC_L_B = 19;
constexpr int PIN_ENC_R_A = 16;
constexpr int PIN_ENC_R_B = 17;
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr int PIN_IMU_INT = 23;
constexpr int PIN_ADC_VBAT = 36;
constexpr int PIN_ADC_CO2 = 39;
constexpr int PIN_ADC_RADAR = 34;
constexpr int PIN_CO2_HEATER_EN = 4;
constexpr int PIN_LED_STATUS = 2;
constexpr uint32_t USB_BAUD = 921600;

// --- Geometría y límites (geometry) ---
constexpr float WHEEL_RADIUS = 0.0325f;
constexpr float WHEEL_SEPARATION = 0.196f;
constexpr float MAX_LIN_VEL = 0.3f;
constexpr float MAX_ANG_VEL = 1.5f;
constexpr float MAX_LIN_ACC = 0.5f;
constexpr float MAX_ANG_ACC = 2.0f;

// --- Motores y encoders (motors) ---
constexpr int32_t COUNTS_PER_WHEEL_REV = 2112;
constexpr float MOTOR_V_RATED = 6.0f;
constexpr float WHEEL_MAX_RAD_S = 20.943951023931955f;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 10;
constexpr float MOTOR_DEADBAND = 0.12f;
constexpr bool ENCODER_USE_PCNT = true;
constexpr uint16_t ENCODER_GLITCH_FILTER = 100;
constexpr bool MOTOR_INVERT_LEFT = false;
constexpr bool MOTOR_INVERT_RIGHT = true;
constexpr bool ENCODER_INVERT_LEFT = false;
constexpr bool ENCODER_INVERT_RIGHT = true;

// --- Batería (battery) ---
constexpr float BAT_DIVIDER_RATIO = 0.2702702702702703f;
constexpr float BAT_V_FULL = 8.4f;
constexpr float BAT_V_LOW = 6.6f;
constexpr float BAT_V_CRITICAL = 6.2f;
constexpr float BAT_V_CUTOFF = 6.0f;
constexpr float BAT_FILTER_ALPHA = 0.2f;
constexpr float BAT_HYSTERESIS = 0.1f;
constexpr int BAT_ADC_ATTEN_DB = 11;

// --- Sensores analógicos (analog) ---
constexpr float CO2_GAIN = 3.0f;
constexpr float CO2_SLOPE_V_PER_DECADE = 0.05f;
constexpr float CO2_REF_PPM = 400.0f;
constexpr float RADAR_VREF = 1.65f;
constexpr int RADAR_RMS_WINDOW = 200;
constexpr int CO2_ADC_ATTEN_DB = 6;
constexpr int RADAR_ADC_ATTEN_DB = 11;

// --- IMU MPU6050 ---
constexpr int IMU_ACCEL_RANGE_G = 2;
constexpr int IMU_GYRO_RANGE_DPS = 250;
constexpr uint8_t IMU_DLPF_CFG = 3;
constexpr int IMU_CALIB_SAMPLES = 200;
constexpr uint8_t IMU_I2C_ADDRESS = 0x68;
constexpr uint32_t IMU_I2C_HZ = 400000;

// --- Control y tiempos (firmware) ---
constexpr uint32_t PERIOD_CONTROL_US = 20000;
constexpr uint32_t PERIOD_IMU_US = 10000;
constexpr uint32_t PERIOD_RADAR_US = 5000;
constexpr uint32_t PERIOD_SLOW_US = 1000000;
constexpr uint32_t PERIOD_TELEMETRY_US = 100000;
constexpr uint32_t CMD_TIMEOUT_MS = 500;
constexpr float PID_KP = 0.02f;
constexpr float PID_KI = 0.2f;
constexpr float PID_KD = 0.0f;
constexpr float PID_I_LIMIT = 0.3f;
constexpr float PID_OUT_LIMIT = 1.0f;

// --- micro-ROS (firmware.ros) ---
constexpr const char* ROS_NODE_NAME = "findbot_esp32";
// FINDBOT_ROS_DOMAIN_E2E lo define solo el entorno esp32_e2e (prueba aislada)
#ifdef FINDBOT_ROS_DOMAIN_E2E
constexpr uint32_t ROS_DOMAIN_ID = 77;
constexpr uint32_t ENTITY_TIMEOUT_MS = 20000;
constexpr int AGENT_TIMEOUT_MS = 3000;
#else
constexpr uint32_t ROS_DOMAIN_ID = 0;
constexpr uint32_t ENTITY_TIMEOUT_MS = 3000;
constexpr int AGENT_TIMEOUT_MS = 1000;
#endif
constexpr const char* TOPIC_CMD_VEL = "/cmd_vel";
constexpr const char* TOPIC_ODOM = "/odom";
constexpr const char* TOPIC_JOINT_STATES = "/joint_states";
constexpr const char* TOPIC_IMU = "/imu/data_raw";
constexpr const char* TOPIC_BATTERY = "/battery_state";
constexpr const char* TOPIC_TF = "/tf";
constexpr const char* FRAME_ODOM = "odom";
constexpr const char* FRAME_BASE = "base_footprint";
constexpr const char* FRAME_IMU = "imu_link";
constexpr const char* JOINT_LEFT = "left_wheel_joint";
constexpr const char* JOINT_RIGHT = "right_wheel_joint";
constexpr uint32_t PUB_PERIOD_ODOM_MS = 50;
constexpr uint32_t PUB_PERIOD_JOINT_STATES_MS = 50;
constexpr uint32_t PUB_PERIOD_IMU_MS = 20;
constexpr uint32_t PUB_PERIOD_BATTERY_MS = 1000;
constexpr bool PUBLISH_ODOM_TF = true;
constexpr uint32_t AGENT_PING_MS = 500;
constexpr uint32_t AGENT_CHECK_MS = 1000;
constexpr uint8_t AGENT_ATTEMPTS = 3;
constexpr int ROS_TASK_CORE = 0;
constexpr uint32_t ROS_TASK_STACK = 16384;
constexpr float COV_ODOM_POSE[6] = {0.001f, 0.001f, 1000000.0f, 1000000.0f, 1000000.0f, 0.01f};
constexpr float COV_ODOM_TWIST[6] = {0.001f, 1000000.0f, 1000000.0f, 1000000.0f, 1000000.0f, 0.01f};
constexpr float COV_IMU_GYRO = 0.0001f;
constexpr float COV_IMU_ACCEL = 0.01f;
constexpr float BAT_DESIGN_CAPACITY_AH = 3.0f;

}  // namespace cfg
