// What to do when the ultrasonic sensor sees something in the way.
//
// The HC-SR04 only looks straight ahead, so the rover uses its whole body
// as a scanner: stop, think for a moment, turn on the spot toward the side
// the next waypoint is on until the echo goes long, turn a bit more so the
// body clears the edge, drive past, then let pure pursuit pull it back onto
// the path. If one side is blocked all the way round, it tries the other.
//
// Low rocks don't trigger any of this. The sensor is mounted high and tilted
// up, so anything the rocker-bogie can climb stays under the beam and the
// rover just drives over it.
#pragma once

#include <cmath>
#include <cstdint>

#include "rover_core/kinematics.hpp"
#include "rover_core/navigation.hpp"
#include "rover_core/params.hpp"

namespace rover {

// Short-term memory of where the sonar has seen something tall. Once the
// rover turns away from an obstacle it can't see it any more, so without
// this it happily drives back into the corner it just avoided. Points are
// only recorded from close echoes (anything seen that close is too tall to
// climb) and fade after a while, because dead reckoning drifts.
class HazardMemory {
 public:
  static constexpr int kMax = 24;
  static constexpr uint32_t kLifetimeMs = 20000;
  static constexpr float kRecordRange = autonomy::kAvoidDistance + 0.05f;
  static constexpr float kMergeRadius = 0.05f;
  static constexpr float kHalfWidth = 0.17f;  // half the wheel track + tyre + a margin
  static constexpr float kSonarX = 0.115f;    // yaml sonar_x

  void clear() { n_ = 0; }
  int size() const { return n_; }

  // Feed every sonar reading; it decides whether it's worth remembering.
  void observe(const Pose2D& p, float range, uint32_t now) {
    if (range <= 0.0f || range > kRecordRange) return;
    const float c = std::cos(p.yaw), s = std::sin(p.yaw);
    const float d = kSonarX + range;
    add(p.x + c * d, p.y + s * d, now);
  }

  // Is anything remembered in the strip the rover is about to drive through?
  bool in_corridor(const Pose2D& p, float length, uint32_t now) const {
    const float c = std::cos(p.yaw), s = std::sin(p.yaw);
    for (int i = 0; i < n_; ++i) {
      if (now - h_[i].t > kLifetimeMs) continue;
      const float dx = h_[i].x - p.x, dy = h_[i].y - p.y;
      const float xr = c * dx + s * dy;
      const float yr = -s * dx + c * dy;
      if (xr > 0.0f && xr < length && std::fabs(yr) < kHalfWidth) return true;
    }
    return false;
  }

 private:
  struct Hazard {
    float x, y;
    uint32_t t;
  };
  void add(float x, float y, uint32_t now) {
    int oldest = 0;
    for (int i = 0; i < n_; ++i) {
      if (std::hypot(h_[i].x - x, h_[i].y - y) < kMergeRadius) {
        h_[i].t = now;
        return;
      }
      if (h_[i].t < h_[oldest].t) oldest = i;
    }
    if (n_ < kMax) {
      h_[n_++] = Hazard{x, y, now};
    } else {
      h_[oldest] = Hazard{x, y, now};
    }
  }
  Hazard h_[kMax] = {};
  int n_ = 0;
};

class Avoidance {
 public:
  enum class State : uint8_t { kClear, kPause, kBackUp, kScan, kExtraTurn, kBypass, kBlocked };

  State state() const { return state_; }
  bool active() const { return state_ != State::kClear; }
  int attempts() const { return attempts_; }
  void reset() {
    state_ = State::kClear;
    attempts_ = 0;
  }
  void new_waypoint() { attempts_ = 0; }

  // Something is wrong but the sonar can't see it (say a wheel is grinding
  // against the corner of a box). Back off and look around as if it had.
  void force(const Pose2D& pose, float goal_side, uint32_t now) {
    ++attempts_;
    dir_ = goal_side >= 0 ? 1.0f : -1.0f;
    tried_other_side_ = false;
    if (attempts_ > autonomy::kMaxAvoidsPerWaypoint) {
      state_ = State::kBlocked;
      t0_ = now;
    } else {
      enter(State::kBackUp, pose, now);
    }
  }

  static const char* name(State s) {
    switch (s) {
      case State::kClear: return "clear";
      case State::kPause: return "pause";
      case State::kBackUp: return "backup";
      case State::kScan: return "scan";
      case State::kExtraTurn: return "extra_turn";
      case State::kBypass: return "bypass";
      case State::kBlocked: return "blocked";
    }
    return "?";
  }

  // range <= 0 means no echo, which counts as clear.
  // goal_side: +1 if the next waypoint is to the left, -1 if to the right.
  // Returns true while avoidance owns the wheels; `out` is then the command.
  bool update(float range, bool range_fresh, const Pose2D& pose, float goal_side, uint32_t now,
              Twist2D& out, const HazardMemory* memory = nullptr) {
    const bool remembered_close =
        memory && memory->in_corridor(pose, HazardMemory::kSonarX + autonomy::kAvoidDistance - 0.1f, now);
    const bool remembered_ahead = memory && memory->in_corridor(pose, 0.65f, now);
    const bool blocked_ahead =
        (range_fresh && range > 0.0f && range < autonomy::kAvoidDistance) || remembered_close;
    const bool clear_ahead =
        range_fresh && (range <= 0.0f || range > autonomy::kClearDistance) && !remembered_ahead;
    out = Twist2D{};
    switch (state_) {
      case State::kClear:
        if (!blocked_ahead) return false;
        ++attempts_;
        if (attempts_ > autonomy::kMaxAvoidsPerWaypoint) {
          state_ = State::kBlocked;
          t0_ = now;
          return true;
        }
        dir_ = goal_side >= 0 ? 1.0f : -1.0f;
        tried_other_side_ = false;
        enter(State::kPause, pose, now);
        return true;

      case State::kPause:
        if (now - t0_ >= autonomy::kPauseMs) {
          enter(range_fresh && range > 0.0f && range < safety::kStopDistance + 0.05f ? State::kBackUp
                                                                                     : State::kScan,
                pose, now);
        }
        return true;

      case State::kBackUp:
        out.v = -0.10f;
        if (travelled(pose) > 0.12f || now - t0_ > 2000) enter(State::kScan, pose, now);
        return true;

      case State::kScan: {
        out.w = dir_ * autonomy::kTurnRate;
        turned_ += std::fabs(wrap_pi(pose.yaw - last_yaw_));
        last_yaw_ = pose.yaw;
        // First side: up to 120 degrees. Other side: swing back past the
        // start and another 120 degrees.
        const float limit = tried_other_side_ ? 4.2f : 2.1f;
        if (clear_ahead && turned_ > 0.15f && !(tried_other_side_ && turned_ < 2.3f)) {
          enter(State::kExtraTurn, pose, now);
        } else if (turned_ > limit) {
          if (tried_other_side_) {
            state_ = State::kBlocked;
            t0_ = now;
          } else {
            tried_other_side_ = true;
            dir_ = -dir_;
            enter(State::kScan, pose, now);
          }
        }
        return true;
      }

      case State::kExtraTurn:
        out.w = dir_ * autonomy::kTurnRate;
        if (std::fabs(wrap_pi(pose.yaw - yaw0_)) > autonomy::kExtraTurn) enter(State::kBypass, pose, now);
        return true;

      case State::kBypass:
        out.v = autonomy::kCruiseSpeed;
        if (blocked_ahead) {
          enter(State::kPause, pose, now);
        } else if (travelled(pose) > autonomy::kBypassDistance) {
          state_ = State::kClear;
          return false;
        }
        return true;

      case State::kBlocked:
        // Sit still. The brain notices and gives up on this waypoint.
        return true;
    }
    return false;
  }

 private:
  void enter(State s, const Pose2D& p, uint32_t now) {
    state_ = s;
    t0_ = now;
    x0_ = p.x;
    y0_ = p.y;
    yaw0_ = p.yaw;
    last_yaw_ = p.yaw;
    turned_ = 0.0f;
  }
  float travelled(const Pose2D& p) const { return std::hypot(p.x - x0_, p.y - y0_); }

  State state_ = State::kClear;
  uint32_t t0_ = 0;
  float x0_ = 0, y0_ = 0, yaw0_ = 0;
  float last_yaw_ = 0, turned_ = 0;
  float dir_ = 1.0f;
  bool tried_other_side_ = false;
  int attempts_ = 0;
};

}  // namespace rover
