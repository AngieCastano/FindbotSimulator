#include "log.h"

#if FINDBOT_MICROROS

#include "ros_bridge.h"

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <tf2_msgs/msg/tf_message.h>

#include "config_gen.h"
#include "ros_convert.h"

namespace {

enum class AgentState { kWaiting, kAvailable, kConnected, kDisconnected };

volatile AgentState g_state = AgentState::kWaiting;
RosBridge::CmdCallback g_on_cmd = nullptr;
uint32_t g_last_ping_ms = 0;
uint32_t g_last_pub_ms[4] = {0, 0, 0, 0};  // odom, joint_states, imu, battery

rclc_support_t g_support;
rcl_allocator_t g_allocator;
rcl_init_options_t g_init_options;
rcl_node_t g_node;
rclc_executor_t g_executor;
rcl_subscription_t g_sub_cmd;
rcl_publisher_t g_pub_odom, g_pub_tf, g_pub_joints, g_pub_imu, g_pub_battery;

geometry_msgs__msg__Twist g_cmd_msg;
nav_msgs__msg__Odometry g_odom_msg;
tf2_msgs__msg__TFMessage g_tf_msg;
sensor_msgs__msg__JointState g_joint_msg;
sensor_msgs__msg__Imu g_imu_msg;
sensor_msgs__msg__BatteryState g_battery_msg;

// Las funciones de rcl están marcadas warn_unused_result y GCC no acepta (void)x para callarlas.
#define RC_CHECK(expr)                    \
  do {                                    \
    const rcl_ret_t rc_ = (expr);         \
    if (rc_ != RCL_RET_OK) return false;  \
  } while (0)
#define RC_IGNORE(expr)            \
  do {                             \
    const rcl_ret_t rc_ = (expr);  \
    (void)rc_;                     \
  } while (0)

void cmd_callback(const void* msgin) {
  const auto* m = static_cast<const geometry_msgs__msg__Twist*>(msgin);
  if (g_on_cmd) g_on_cmd(static_cast<float>(m->linear.x), static_cast<float>(m->angular.z));
}

void stamp(builtin_interfaces__msg__Time& t) {
  int32_t sec = 0;
  uint32_t nsec = 0;
  core::split_nanos(rmw_uros_epoch_nanos(), sec, nsec);
  t.sec = sec;
  t.nanosec = nsec;
}

// Memoria de los mensajes: se reserva una sola vez (cadenas y secuencias) y se reutiliza.
void init_messages() {
  nav_msgs__msg__Odometry__init(&g_odom_msg);
  rosidl_runtime_c__String__assign(&g_odom_msg.header.frame_id, cfg::FRAME_ODOM);
  rosidl_runtime_c__String__assign(&g_odom_msg.child_frame_id, cfg::FRAME_BASE);
  core::covariance6_from_diagonal(cfg::COV_ODOM_POSE, g_odom_msg.pose.covariance);
  core::covariance6_from_diagonal(cfg::COV_ODOM_TWIST, g_odom_msg.twist.covariance);

  tf2_msgs__msg__TFMessage__init(&g_tf_msg);
  geometry_msgs__msg__TransformStamped__Sequence__init(&g_tf_msg.transforms, 1);
  rosidl_runtime_c__String__assign(&g_tf_msg.transforms.data[0].header.frame_id, cfg::FRAME_ODOM);
  rosidl_runtime_c__String__assign(&g_tf_msg.transforms.data[0].child_frame_id, cfg::FRAME_BASE);

  sensor_msgs__msg__JointState__init(&g_joint_msg);
  rosidl_runtime_c__String__Sequence__init(&g_joint_msg.name, 2);
  rosidl_runtime_c__String__assign(&g_joint_msg.name.data[0], cfg::JOINT_LEFT);
  rosidl_runtime_c__String__assign(&g_joint_msg.name.data[1], cfg::JOINT_RIGHT);
  rosidl_runtime_c__double__Sequence__init(&g_joint_msg.position, 2);
  rosidl_runtime_c__double__Sequence__init(&g_joint_msg.velocity, 2);

  sensor_msgs__msg__Imu__init(&g_imu_msg);
  rosidl_runtime_c__String__assign(&g_imu_msg.header.frame_id, cfg::FRAME_IMU);
  g_imu_msg.orientation_covariance[0] = -1.0;  // REP 145: el MPU6050 crudo no da orientación
  core::covariance3_isotropic(cfg::COV_IMU_GYRO, g_imu_msg.angular_velocity_covariance);
  core::covariance3_isotropic(cfg::COV_IMU_ACCEL, g_imu_msg.linear_acceleration_covariance);

  sensor_msgs__msg__BatteryState__init(&g_battery_msg);
  g_battery_msg.present = true;
  g_battery_msg.power_supply_status = core::kPowerSupplyStatusDischarging;
  g_battery_msg.power_supply_technology = core::kPowerSupplyTechnologyLion;
  g_battery_msg.design_capacity = cfg::BAT_DESIGN_CAPACITY_AH;
  g_battery_msg.current = NAN;
  g_battery_msg.charge = NAN;
  g_battery_msg.capacity = NAN;
  g_battery_msg.temperature = NAN;

  geometry_msgs__msg__Twist__init(&g_cmd_msg);
}

bool create_entities() {
  g_allocator = rcl_get_default_allocator();
  // Dominio DDS explícito: el microcontrolador no tiene variable de entorno ROS_DOMAIN_ID
  g_init_options = rcl_get_zero_initialized_init_options();
  RC_CHECK(rcl_init_options_init(&g_init_options, g_allocator));
  RC_CHECK(rcl_init_options_set_domain_id(&g_init_options, cfg::ROS_DOMAIN_ID));
  RC_CHECK(rclc_support_init_with_options(&g_support, 0, nullptr, &g_init_options, &g_allocator));
  // Con mucha latencia (Wokwi pasa por la nube) la creación de entidades necesita más tiempo
  RC_IGNORE(rmw_uros_set_context_entity_creation_session_timeout(
      rcl_context_get_rmw_context(&g_support.context), cfg::ENTITY_TIMEOUT_MS));
  RC_CHECK(rclc_node_init_default(&g_node, cfg::ROS_NODE_NAME, "", &g_support));
  RC_CHECK(rclc_publisher_init_default(&g_pub_odom, &g_node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), cfg::TOPIC_ODOM));
  if (cfg::PUBLISH_ODOM_TF) {
    RC_CHECK(rclc_publisher_init_default(&g_pub_tf, &g_node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), cfg::TOPIC_TF));
  }
  RC_CHECK(rclc_publisher_init_default(&g_pub_joints, &g_node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                       cfg::TOPIC_JOINT_STATES));
  // IMU a alta frecuencia: best effort (si se pierde una muestra, la siguiente llega en 20 ms)
  RC_CHECK(rclc_publisher_init_best_effort(&g_pub_imu, &g_node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), cfg::TOPIC_IMU));
  RC_CHECK(rclc_publisher_init_default(&g_pub_battery, &g_node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
                                       cfg::TOPIC_BATTERY));
  RC_CHECK(rclc_subscription_init_default(&g_sub_cmd, &g_node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                          cfg::TOPIC_CMD_VEL));
  g_executor = rclc_executor_get_zero_initialized_executor();
  RC_CHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));
  RC_CHECK(rclc_executor_add_subscription(&g_executor, &g_sub_cmd, &g_cmd_msg, &cmd_callback, ON_NEW_DATA));
  // Sincroniza el reloj con el agente para que los mensajes lleven la hora de ROS
  RC_IGNORE(rmw_uros_sync_session(cfg::AGENT_TIMEOUT_MS));
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&g_support.context);
  RC_IGNORE(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));
  RC_IGNORE(rcl_subscription_fini(&g_sub_cmd, &g_node));
  RC_IGNORE(rcl_publisher_fini(&g_pub_odom, &g_node));
  if (cfg::PUBLISH_ODOM_TF) RC_IGNORE(rcl_publisher_fini(&g_pub_tf, &g_node));
  RC_IGNORE(rcl_publisher_fini(&g_pub_joints, &g_node));
  RC_IGNORE(rcl_publisher_fini(&g_pub_imu, &g_node));
  RC_IGNORE(rcl_publisher_fini(&g_pub_battery, &g_node));
  RC_IGNORE(rclc_executor_fini(&g_executor));
  RC_IGNORE(rcl_node_fini(&g_node));
  RC_IGNORE(rclc_support_fini(&g_support));
  RC_IGNORE(rcl_init_options_fini(&g_init_options));
}

bool due(int idx, uint32_t period_ms, uint32_t now) {
  if (now - g_last_pub_ms[idx] < period_ms) return false;
  g_last_pub_ms[idx] = now;
  return true;
}

void publish(const RosSnapshot& s, uint32_t now) {
  if (due(0, cfg::PUB_PERIOD_ODOM_MS, now)) {
    const core::Quaternion q = core::yaw_to_quaternion(s.theta);
    stamp(g_odom_msg.header.stamp);
    g_odom_msg.pose.pose.position.x = s.x;
    g_odom_msg.pose.pose.position.y = s.y;
    g_odom_msg.pose.pose.orientation.z = q.z;
    g_odom_msg.pose.pose.orientation.w = q.w;
    g_odom_msg.twist.twist.linear.x = s.v;
    g_odom_msg.twist.twist.angular.z = s.w;
    RC_IGNORE(rcl_publish(&g_pub_odom, &g_odom_msg, nullptr));
    if (cfg::PUBLISH_ODOM_TF) {
      geometry_msgs__msg__TransformStamped& t = g_tf_msg.transforms.data[0];
      t.header.stamp = g_odom_msg.header.stamp;
      t.transform.translation.x = s.x;
      t.transform.translation.y = s.y;
      t.transform.rotation.z = q.z;
      t.transform.rotation.w = q.w;
      RC_IGNORE(rcl_publish(&g_pub_tf, &g_tf_msg, nullptr));
    }
  }
  if (due(1, cfg::PUB_PERIOD_JOINT_STATES_MS, now)) {
    stamp(g_joint_msg.header.stamp);
    g_joint_msg.position.data[0] = s.wheel_pos_left;
    g_joint_msg.position.data[1] = s.wheel_pos_right;
    g_joint_msg.velocity.data[0] = s.wheel_vel_left;
    g_joint_msg.velocity.data[1] = s.wheel_vel_right;
    RC_IGNORE(rcl_publish(&g_pub_joints, &g_joint_msg, nullptr));
  }
  if (s.imu_ok && due(2, cfg::PUB_PERIOD_IMU_MS, now)) {
    stamp(g_imu_msg.header.stamp);
    g_imu_msg.angular_velocity.x = s.imu.gx;
    g_imu_msg.angular_velocity.y = s.imu.gy;
    g_imu_msg.angular_velocity.z = s.imu.gz;
    g_imu_msg.linear_acceleration.x = s.imu.ax;
    g_imu_msg.linear_acceleration.y = s.imu.ay;
    g_imu_msg.linear_acceleration.z = s.imu.az;
    RC_IGNORE(rcl_publish(&g_pub_imu, &g_imu_msg, nullptr));
  }
  if (due(3, cfg::PUB_PERIOD_BATTERY_MS, now)) {
    stamp(g_battery_msg.header.stamp);
    g_battery_msg.voltage = s.battery_voltage;
    g_battery_msg.percentage = s.battery_percentage;
    g_battery_msg.power_supply_health = core::battery_health(s.battery_level);
    RC_IGNORE(rcl_publish(&g_pub_battery, &g_battery_msg, nullptr));
  }
}

}  // namespace

void RosBridge::begin(CmdCallback on_cmd) {
  g_on_cmd = on_cmd;
  set_microros_serial_transports(Serial);  // Serial.begin(USB_BAUD) ya lo hizo setup()
  init_messages();
}

bool RosBridge::spin(const RosSnapshot& s) {
  const uint32_t now = millis();
  switch (g_state) {
    case AgentState::kWaiting:
      if (now - g_last_ping_ms >= cfg::AGENT_PING_MS) {
        g_last_ping_ms = now;
        if (rmw_uros_ping_agent(cfg::AGENT_TIMEOUT_MS, 1) == RMW_RET_OK) g_state = AgentState::kAvailable;
      }
      break;
    case AgentState::kAvailable:
      if (create_entities()) {
        g_state = AgentState::kConnected;
      } else {
        destroy_entities();
        g_state = AgentState::kWaiting;
      }
      break;
    case AgentState::kConnected:
      if (now - g_last_ping_ms >= cfg::AGENT_CHECK_MS) {
        g_last_ping_ms = now;
        if (rmw_uros_ping_agent(cfg::AGENT_TIMEOUT_MS, cfg::AGENT_ATTEMPTS) != RMW_RET_OK) {
          g_state = AgentState::kDisconnected;
          break;
        }
      }
      RC_IGNORE(rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(0)));
      publish(s, now);
      break;
    case AgentState::kDisconnected:
      destroy_entities();
      g_state = AgentState::kWaiting;
      break;
  }
  return g_state == AgentState::kConnected;
}

bool RosBridge::connected() const { return g_state == AgentState::kConnected; }

#endif  // FINDBOT_MICROROS
