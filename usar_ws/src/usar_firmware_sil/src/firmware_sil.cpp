// Nodo firmware_sil: el firmware del ESP32 en el lazo con Gazebo (software-in-the-loop).
//
// Corre, en el PC y en tiempo simulado, el mismo lazo de control que hardware/firmware/src/
// main.cpp (control_step) con las clases de hardware/firmware/lib/robot_core compiladas tal cual:
//   /cmd_vel → CommandWatchdog → VelocityLimiter → inverse_kinematics → Pid por rueda
//   (+ prealimentación) → duty_from_command → [modelo del motor TT] → par en cada rueda (Gazebo)
//   encoders (ángulo de la rueda simulada, cuantizado a COUNTS_PER_WHEEL_REV) → core::Odometry
//   → /odom
// Constantes (periodo, ganancias, límites, tiempo del vigilante) de config_gen.h, generado desde
// hardware/config/robot_hw.yaml: las mismas que el firmware del robot.
//
// Lo que NO es el firmware: el modelo del motor y del encoder (física que Gazebo no tiene para
// estos actuadores) y la orquestación del lazo, que reproduce la de main.cpp porque esta depende
// de Arduino (millis, PCNT, LEDC, portMUX). La batería se supone constante (battery_voltage).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "config_gen.h"
#include "kinematics.h"
#include "motor_limit.h"
#include "pid.h"
#include "watchdog.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "usar_firmware_sil/motor_model.hpp"

namespace {

const core::DiffDriveParams kDrive{cfg::WHEEL_RADIUS, cfg::WHEEL_SEPARATION, cfg::MAX_LIN_VEL,
                                   cfg::MAX_ANG_VEL,  cfg::MAX_LIN_ACC,      cfg::MAX_ANG_ACC};
const core::PidGains kPid{cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, cfg::PID_I_LIMIT,
                          cfg::PID_OUT_LIMIT};

}  // namespace

class FirmwareSil : public rclcpp::Node {
 public:
  FirmwareSil()
  : Node("firmware_sil"),
    watchdog_(cfg::CMD_TIMEOUT_MS),
    limiter_(kDrive),
    odom_(kDrive, cfg::COUNTS_PER_WHEEL_REV),
    pid_left_(kPid),
    pid_right_(kPid)
  {
    battery_ = declare_parameter("battery_voltage", 7.8);
    motor_.v_test = declare_parameter("motor_v_test", 6.0);
    motor_.no_load_speed = declare_parameter("motor_no_load_speed", 26.18);
    motor_.stall_torque = declare_parameter("motor_stall_torque", 0.0785);
    motor_.friction_torque = declare_parameter("motor_friction_torque", 0.008);
    left_joint_ = declare_parameter("left_joint", std::string("left_wheel_joint"));
    right_joint_ = declare_parameter("right_joint", std::string("right_wheel_joint"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
    const double odom_period = declare_parameter(
      "odom_period", cfg::PUB_PERIOD_ODOM_MS / 1000.0);
    cov_v_ = declare_parameter("twist_variance_linear", 2.5e-5);
    cov_w_ = declare_parameter("twist_variance_angular", 1e-4);
    period_ = cfg::PERIOD_CONTROL_US * 1e-6;

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        // on_velocity_command() de main.cpp
        cmd_ = {static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z)};
        watchdog_.feed(now_ms());
        had_command_ = true;
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joints(*msg);});
    torque_left_ = create_publisher<std_msgs::msg::Float64>("firmware/left_wheel_torque", 10);
    torque_right_ = create_publisher<std_msgs::msg::Float64>("firmware/right_wheel_torque", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    state_pub_ = create_publisher<std_msgs::msg::String>("firmware/state", 10);

    // Temporizadores en tiempo simulado (use_sim_time): el lazo corre al ritmo de Gazebo.
    control_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(period_), [this]() {control_step();});
    odom_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(odom_period), [this]() {publish_odom();});
    RCLCPP_INFO(
      get_logger(), "Firmware en el lazo: control cada %.0f ms, vigilante %u ms, PID kp=%.3f "
      "ki=%.3f, %d cuentas/vuelta, batería %.1f V", period_ * 1e3,
      static_cast<unsigned>(cfg::CMD_TIMEOUT_MS), cfg::PID_KP, cfg::PID_KI,
      static_cast<int>(cfg::COUNTS_PER_WHEEL_REV), battery_);
  }

 private:
  uint32_t now_ms() {return static_cast<uint32_t>(now().nanoseconds() / 1000000);}

  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      const double pos = i < msg.position.size() ? msg.position[i] : 0.0;
      const double vel = i < msg.velocity.size() ? msg.velocity[i] : 0.0;
      if (msg.name[i] == left_joint_) {
        angle_left_ = pos;
        omega_left_ = vel;
        have_joints_ = true;
      } else if (msg.name[i] == right_joint_) {
        angle_right_ = pos;
        omega_right_ = vel;
      }
    }
  }

  // Encoders: variación de cuentas desde el periodo anterior (read_deltas() del firmware).
  void read_deltas(int32_t & dl, int32_t & dr)
  {
    const int32_t tl = sil::encoder_ticks(angle_left_, cfg::COUNTS_PER_WHEEL_REV);
    const int32_t tr = sil::encoder_ticks(angle_right_, cfg::COUNTS_PER_WHEEL_REV);
    if (!ticks_init_) {
      last_ticks_left_ = tl;
      last_ticks_right_ = tr;
      ticks_init_ = true;
    }
    dl = tl - last_ticks_left_;
    dr = tr - last_ticks_right_;
    last_ticks_left_ = tl;
    last_ticks_right_ = tr;
  }

  void set_state(const char * state)
  {
    if (state_ == state) {return;}
    state_ = state;
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Estado: %s", state);
  }

  // control_step() de hardware/firmware/src/main.cpp (sin la rama de batería crítica).
  void control_step()
  {
    if (!have_joints_) {return;}
    const float dt = static_cast<float>(period_);
    int32_t dl = 0, dr = 0;
    read_deltas(dl, dr);
    odom_.update(dl, dr, dt);

    const bool stale = watchdog_.expired(now_ms());
    const core::Twist2D target = stale ? core::Twist2D{0.0f, 0.0f} : cmd_;
    const core::Twist2D limited = limiter_.update(target, dt);
    if (stale) {
      set_state(had_command_ ? "stale" : "idle");
    } else if (target.v == 0.0f && target.w == 0.0f) {
      set_state("idle");
    } else {
      set_state("driving");
    }

    float duty_left = 0.0f, duty_right = 0.0f;
    if (limited.v == 0.0f && limited.w == 0.0f) {
      pid_left_.reset();
      pid_right_.reset();
    } else {
      const core::WheelSpeeds want = core::inverse_kinematics(kDrive, limited);
      const core::WheelSpeeds meas = odom_.wheel_speeds();
      const float u_left = want.left / cfg::WHEEL_MAX_RAD_S +
        pid_left_.update(want.left, meas.left, dt);
      const float u_right = want.right / cfg::WHEEL_MAX_RAD_S +
        pid_right_.update(want.right, meas.right, dt);
      const float v_bat = static_cast<float>(battery_);
      duty_left = core::duty_from_command(u_left, v_bat, cfg::MOTOR_V_LIMIT, cfg::MOTOR_DEADBAND);
      duty_right = core::duty_from_command(
        u_right, v_bat, cfg::MOTOR_V_LIMIT, cfg::MOTOR_DEADBAND);
    }
    // Motores (TB6612 + TT 1:48): duty → par en cada rueda de Gazebo.
    std_msgs::msg::Float64 tl, tr;
    tl.data = sil::wheel_torque(motor_, duty_left, battery_, omega_left_);
    tr.data = sil::wheel_torque(motor_, duty_right, battery_, omega_right_);
    torque_left_->publish(tl);
    torque_right_->publish(tr);
  }

  void publish_odom()
  {
    if (!have_joints_) {return;}
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = odom_.x();
    msg.pose.pose.position.y = odom_.y();
    msg.pose.pose.orientation.z = std::sin(odom_.theta() / 2.0);
    msg.pose.pose.orientation.w = std::cos(odom_.theta() / 2.0);
    msg.twist.twist.linear.x = odom_.v();
    msg.twist.twist.angular.z = odom_.w();
    const double big = 1e6;  // z, roll y pitch no se estiman (robot planar)
    const double pose_cov[6] = {1e-3, 1e-3, big, big, big, 1e-2};
    const double twist_cov[6] = {cov_v_, cov_v_, big, big, big, cov_w_};
    for (int i = 0; i < 6; ++i) {
      msg.pose.covariance[7 * i] = pose_cov[i];
      msg.twist.covariance[7 * i] = twist_cov[i];
    }
    odom_pub_->publish(msg);
  }

  core::CommandWatchdog watchdog_;
  core::VelocityLimiter limiter_;
  core::Odometry odom_;
  core::Pid pid_left_, pid_right_;
  sil::MotorParams motor_{};
  core::Twist2D cmd_{0.0f, 0.0f};
  bool had_command_ = false;
  double battery_ = 7.8, period_ = 0.02, cov_v_ = 0.0, cov_w_ = 0.0;
  std::string left_joint_, right_joint_, odom_frame_, base_frame_, state_;
  double angle_left_ = 0.0, angle_right_ = 0.0, omega_left_ = 0.0, omega_right_ = 0.0;
  bool have_joints_ = false, ticks_init_ = false;
  int32_t last_ticks_left_ = 0, last_ticks_right_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_left_, torque_right_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_, odom_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirmwareSil>());
  rclcpp::shutdown();
  return 0;
}
