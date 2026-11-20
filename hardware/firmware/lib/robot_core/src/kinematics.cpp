#include "kinematics.h"

#include "util.h"

namespace core {

Twist2D VelocityLimiter::update(Twist2D target, float dt) {
  target.v = clampf(target.v, -p_.max_lin_vel, p_.max_lin_vel);
  target.w = clampf(target.w, -p_.max_ang_vel, p_.max_ang_vel);
  if (dt > 0.0f) {
    const float dv = p_.max_lin_acc * dt;
    const float dw = p_.max_ang_acc * dt;
    target.v = clampf(target.v, last_.v - dv, last_.v + dv);
    target.w = clampf(target.w, last_.w - dw, last_.w + dw);
  }
  last_ = target;
  return target;
}

WheelSpeeds inverse_kinematics(const DiffDriveParams& p, Twist2D cmd) {
  const float half = 0.5f * p.wheel_separation;
  return {(cmd.v - cmd.w * half) / p.wheel_radius, (cmd.v + cmd.w * half) / p.wheel_radius};
}

Twist2D forward_kinematics(const DiffDriveParams& p, WheelSpeeds w) {
  const float vl = w.left * p.wheel_radius;
  const float vr = w.right * p.wheel_radius;
  return {0.5f * (vl + vr), (vr - vl) / p.wheel_separation};
}

void Odometry::update(int32_t d_ticks_left, int32_t d_ticks_right, float dt) {
  const float dphi_l = ticks_to_rad(d_ticks_left, cpr_);
  const float dphi_r = ticks_to_rad(d_ticks_right, cpr_);
  positions_.left += dphi_l;
  positions_.right += dphi_r;

  const float dl = dphi_l * p_.wheel_radius;
  const float dr = dphi_r * p_.wheel_radius;
  const float ds = 0.5f * (dl + dr);
  const float dth = (dr - dl) / p_.wheel_separation;
  // Integración por punto medio (exacta para un arco con ω constante en primer orden)
  const float mid = theta_ + 0.5f * dth;
  x_ += ds * cosf(mid);
  y_ += ds * sinf(mid);
  theta_ = wrap_angle(theta_ + dth);

  if (dt > 0.0f) {
    speeds_ = {dphi_l / dt, dphi_r / dt};
    v_ = ds / dt;
    w_ = dth / dt;
  }
}

void Odometry::reset() {
  x_ = y_ = theta_ = v_ = w_ = 0.0f;
  speeds_ = {0.0f, 0.0f};
  positions_ = {0.0f, 0.0f};
}

}  // namespace core
