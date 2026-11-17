// The STM32's job, without any hardware in it: take drive commands from the
// ESP32 and the latest ultrasonic range, and decide the PWM duty for each
// of the six motors and the angle for the two steering servos.
//
// It is deliberately paranoid. If the commands stop coming, the wheels stop.
// If something is close in front, forward motion is refused no matter who
// asked for it. Speed changes are ramped so the gearboxes and the printed
// joints don't take a beating.
//
// The same class runs in the simulator (rover_sil), so what you test on the
// laptop is what runs on the robot.
#pragma once

#include <cmath>
#include <cstdint>

#include "rover_core/kinematics.hpp"
#include "rover_core/params.hpp"
#include "rover_core/protocol.hpp"

namespace rover {

struct BodyOutput {
  float duty[geo::kWheels] = {0, 0, 0, 0, 0, 0};  // -1..1, sign = direction, 0 = brake
  float steer_target[2] = {0, 0};                 // what to send the servos (rad)
  float steer_estimate[2] = {0, 0};               // where we think the servos are
  Twist2D applied;                                // best guess of what the rover is doing
  uint8_t status = 0;                             // proto::StatusFlags
};

class BodyController {
 public:
  void on_command(const proto::DriveCommand& c, uint32_t now_ms) {
    cmd_.v = c.v_mm_s * 0.001f;
    cmd_.w = c.w_mrad_s * 0.001f;
    flags_ = c.flags;
    last_cmd_ms_ = now_ms;
    have_cmd_ = true;
  }

  // range_m <= 0 means no echo (nothing within ~4 m, or the sensor is unplugged).
  void on_range(float range_m, uint32_t now_ms) {
    range_ = range_m;
    last_range_ms_ = now_ms;
    have_range_ = true;
  }

  float range() const { return range_; }
  bool range_fresh(uint32_t now_ms) const {
    return have_range_ && (now_ms - last_range_ms_) < safety::kRangeStaleMs;
  }

  BodyOutput step(uint32_t now_ms, float dt) {
    BodyOutput out;
    Twist2D target = cmd_;
    uint8_t flags = flags_;

    if (!have_cmd_ || (now_ms - last_cmd_ms_) > safety::kCommandTimeoutMs) {
      target = Twist2D{};
      flags = 0;
      out.status |= proto::kStatusTimeout;
    }
    target.v = clampf(target.v, -motor::kMaxSpeed, motor::kMaxSpeed);
    target.w = clampf(target.w, -safety::kMaxTurnRate, safety::kMaxTurnRate);

    // --- Ultrasonic reflex -------------------------------------------------
    bool hard_stop = false;
    const bool fresh = range_fresh(now_ms);
    if (fresh) out.status |= proto::kStatusRangeValid;
    if (fresh && range_ > 0.0f && !(flags & proto::kFlagNoReflex) && target.v > 0.0f) {
      if (range_ < safety::kStopDistance) {
        target.v = 0.0f;
        hard_stop = true;
        out.status |= proto::kStatusReflexStop;
      } else if (range_ < safety::kSlowDistance) {
        const float k = (range_ - safety::kStopDistance) / (safety::kSlowDistance - safety::kStopDistance);
        target.v *= k;
        out.status |= proto::kStatusReflexSlow;
      }
    }

    // --- Ramps (braking is allowed to be three times harder) ---------------
    ramp(v_, target.v, safety::kMaxAccel * dt, hard_stop);
    ramp(w_, target.w, safety::kMaxAngAccel * dt, false);
    if (hard_stop && v_ > 0.0f) v_ = 0.0f;

    // --- Kinematics ----------------------------------------------------------
    WheelCommand wc = inverse_kinematics(Twist2D{v_, w_});
    if (wc.steer_valid) {
      steer_target_[0] = wc.steer[0];
      steer_target_[1] = wc.steer[1];
    }
    float worst_err = 0.0f;
    for (int k = 0; k < 2; ++k) {
      const float step_max = geo::kSteerRate * dt;
      const float err = steer_target_[k] - steer_est_[k];
      steer_est_[k] += clampf(err, -step_max, step_max);
      worst_err = std::fmax(worst_err, std::fabs(steer_target_[k] - steer_est_[k]));
      out.steer_target[k] = steer_target_[k];
      out.steer_estimate[k] = steer_est_[k];
    }
    // Like the real rovers: point the wheels first, then drive.
    float hold = 1.0f;
    if (worst_err > 0.12f) {
      hold = clampf(1.0f - (worst_err - 0.12f) / 0.25f, 0.0f, 1.0f);
      out.status |= proto::kStatusSteering;
    }

    // --- Wheel speed -> duty -------------------------------------------------
    float biggest = 0.0f;
    for (int i = 0; i < geo::kWheels; ++i) biggest = std::fmax(biggest, std::fabs(wc.speed[i]));
    const bool moving = biggest > 0.02f * motor::kMaxWheelSpeed;
    const bool climb = (flags & proto::kFlagClimb) != 0;
    if (climb) out.status |= proto::kStatusClimb;

    float scale = 1.0f;  // how much faster/slower the wheels run than the twist asked for
    if (moving) {
      if (climb) {
        // Open loop, a DC motor only has torque to spare if it has voltage
        // to spare. So in climb mode the fastest wheel always gets
        // kClimbDuty and the others keep their ratio to it.
        const float w_at_climb_duty =
            motor::kMaxWheelSpeed * (motor::kClimbDuty - motor::kStartDuty) / (1.0f - motor::kStartDuty);
        scale = w_at_climb_duty / biggest;
      } else if (biggest > motor::kMaxWheelSpeed) {
        scale = motor::kMaxWheelSpeed / biggest;  // keep the curvature, lose some speed
      }
      scale *= hold;
      for (int i = 0; i < geo::kWheels; ++i) out.duty[i] = duty_for(wc.speed[i] * scale);
    }
    out.applied = moving ? Twist2D{v_ * scale, w_ * scale} : Twist2D{};
    return out;
  }

  // Duty for a wheel speed with no load, from V = R*I0 + ke*w. Small
  // speeds round to zero (brake) rather than sitting in the dead band.
  static float duty_for(float wheel_speed) {
    const float a = std::fabs(wheel_speed);
    if (a < 0.03f * motor::kMaxWheelSpeed) return 0.0f;
    float d = motor::kStartDuty + (1.0f - motor::kStartDuty) * a / motor::kMaxWheelSpeed;
    d = clampf(d, 0.0f, 1.0f);
    return wheel_speed >= 0 ? d : -d;
  }

 private:
  static void ramp(float& x, float target, float step, bool instant_brake) {
    const bool braking = std::fabs(target) < std::fabs(x) || (target * x < 0.0f);
    if (instant_brake && braking) {
      x = target;
      return;
    }
    const float s = braking ? 3.0f * step : step;
    x += clampf(target - x, -s, s);
  }

  Twist2D cmd_;
  uint8_t flags_ = 0;
  uint32_t last_cmd_ms_ = 0, last_range_ms_ = 0;
  bool have_cmd_ = false, have_range_ = false;
  float range_ = 0.0f;
  float v_ = 0.0f, w_ = 0.0f;
  float steer_target_[2] = {0, 0};
  float steer_est_[2] = {0, 0};
};

}  // namespace rover
