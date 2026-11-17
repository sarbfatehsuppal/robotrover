// Turning a body velocity into six wheel speeds and two steering angles.
//
// Only the front corners have servos, so the turning centre (ICR) sits on a
// fixed line across the rover, halfway between the middle and rear axles.
// Every wheel's speed comes from its distance to that point, and the two
// front wheels point along the circle they're on, like the Curiosity
// rover does with its corner wheels.
#pragma once

#include <cmath>

#include "rover_core/params.hpp"

namespace rover {

struct Twist2D {
  float v = 0.0f;  // forward speed of base_link, m/s
  float w = 0.0f;  // yaw rate, rad/s, counter-clockwise positive
};

struct BodyTwist {
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
};

struct WheelCommand {
  float speed[geo::kWheels] = {0, 0, 0, 0, 0, 0};  // wheel angular speed, rad/s
  float steer[2] = {0, 0};                         // FL, FR, rad, positive = toe to the left
  bool steer_valid = true;                         // false when standing still (keep the old angles)
};

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

inline float wrap_pi(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

// Velocity of the point (x, y) on the body for the commanded twist.
// The rover turns about (kIcrX, v/w), so base_link drifts sideways a touch
// when turning; that's real and odometry accounts for it.
inline void point_velocity(const Twist2D& t, float x, float y, float& vx, float& vy) {
  vx = t.v - t.w * y;
  vy = t.w * (x - geo::kIcrX);
}

inline BodyTwist body_twist(const Twist2D& t) {
  BodyTwist b;
  point_velocity(t, 0.0f, 0.0f, b.vx, b.vy);
  b.w = t.w;
  return b;
}

inline WheelCommand inverse_kinematics(const Twist2D& t) {
  WheelCommand out;
  const bool still = std::fabs(t.v) < 1e-3f && std::fabs(t.w) < 1e-3f;
  out.steer_valid = !still;
  for (int i = 0; i < geo::kWheels; ++i) {
    float vx, vy;
    point_velocity(t, geo::kWheelX[i], geo::kWheelY[i], vx, vy);
    const bool steerable = (i == geo::kFL || i == geo::kFR);
    if (!steerable) {
      // Fixed wheels roll with the forward part of their velocity and
      // scrub the sideways part.
      out.speed[i] = vx / geo::kWheelRadius;
      continue;
    }
    float angle = std::atan2(vy, vx);
    float speed = std::sqrt(vx * vx + vy * vy);
    // Never point a wheel more than 90 degrees off; drive it backwards instead.
    if (angle > 0.5f * kPi) {
      angle -= kPi;
      speed = -speed;
    } else if (angle < -0.5f * kPi) {
      angle += kPi;
      speed = -speed;
    }
    const float limited = clampf(angle, -geo::kSteerLimit, geo::kSteerLimit);
    // If the servo can't reach the ideal angle, only the in-line part of
    // the wheel's velocity is any use.
    speed *= std::cos(angle - limited);
    out.speed[i] = speed / geo::kWheelRadius;
    out.steer[i == geo::kFL ? 0 : 1] = limited;
  }
  return out;
}

}  // namespace rover
