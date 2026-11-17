// Dead-reckoning odometry and a pure-pursuit waypoint follower.
//
// There are no wheel encoders on this rover, so odometry integrates what
// the STM32 says it commanded. That's fine on a flat floor and drifts when
// the wheels slip or slow down on a climb. Expect tens of centimetres of
// error over a few metres; docs/physics.md has the reasoning and the fix
// (the TT motors have a second shaft just begging for an encoder disc).
#pragma once

#include <cmath>
#include <cstdint>

#include "rover_core/kinematics.hpp"
#include "rover_core/params.hpp"

namespace rover {

struct Pose2D {
  float x = 0.0f, y = 0.0f, yaw = 0.0f;
};

struct Waypoint {
  float x = 0.0f, y = 0.0f;
};

class Odometry {
 public:
  void reset(const Pose2D& p = Pose2D{}) { pose_ = p; }
  void set(const Pose2D& p) { pose_ = p; }
  const Pose2D& pose() const { return pose_; }
  const BodyTwist& twist() const { return twist_; }

  void integrate(const Twist2D& t, float dt) {
    if (dt <= 0.0f || dt > 0.5f) return;
    twist_ = body_twist(t);
    const float mid = pose_.yaw + 0.5f * twist_.w * dt;
    const float c = std::cos(mid), s = std::sin(mid);
    pose_.x += (twist_.vx * c - twist_.vy * s) * dt;
    pose_.y += (twist_.vx * s + twist_.vy * c) * dt;
    pose_.yaw = wrap_pi(pose_.yaw + twist_.w * dt);
  }

 private:
  Pose2D pose_;
  BodyTwist twist_;
};

class Mission {
 public:
  void set(const Waypoint* wps, int n, bool loop, uint32_t id, const Pose2D& start) {
    n_ = n < 0 ? 0 : (n > autonomy::kMaxWaypoints ? autonomy::kMaxWaypoints : n);
    for (int i = 0; i < n_; ++i) wps_[i] = wps[i];
    loop_ = loop;
    id_ = id;
    restart(start);
  }

  void restart(const Pose2D& start) {
    index_ = 0;
    prev_ = Waypoint{start.x, start.y};
    done_ = (n_ == 0);
  }

  int size() const { return n_; }
  int index() const { return index_; }
  bool done() const { return done_; }
  uint32_t id() const { return id_; }
  bool loop() const { return loop_; }
  const Waypoint& at(int i) const { return wps_[i]; }
  const Waypoint& current() const { return wps_[index_ < n_ ? index_ : n_ - 1]; }
  const Waypoint& previous() const { return prev_; }
  bool last_leg() const { return !loop_ && index_ == n_ - 1; }

  void advance() {
    if (done_) return;
    prev_ = wps_[index_];
    ++index_;
    if (index_ >= n_) {
      if (loop_) {
        index_ = 0;
      } else {
        index_ = n_ - 1;
        done_ = true;
      }
    }
  }

  // After a detour, head straight for the waypoint from wherever we ended up
  // rather than steering back onto the old line (which goes through the
  // thing we just avoided).
  void rebase(const Pose2D& p) { prev_ = Waypoint{p.x, p.y}; }

  // After a detour, skip waypoints we've effectively passed.
  void skip_passed(const Pose2D& p) {
    while (!done_ && index_ + 1 < n_) {
      const Waypoint& a = wps_[index_];
      const Waypoint& b = wps_[index_ + 1];
      const float da = std::hypot(a.x - p.x, a.y - p.y);
      const float db = std::hypot(b.x - p.x, b.y - p.y);
      const float ab = std::hypot(b.x - a.x, b.y - a.y);
      if (db < ab && da > autonomy::kLookahead && db < da) {
        advance();
      } else {
        break;
      }
    }
  }

 private:
  Waypoint wps_[autonomy::kMaxWaypoints];
  int n_ = 0, index_ = 0;
  bool loop_ = false, done_ = true;
  uint32_t id_ = 0;
  Waypoint prev_;
};

// Pure pursuit: chase a point one lookahead distance down the current leg.
// If that point is well off to the side or behind, stop and turn on the
// spot first. Slower overall, but much tidier with front-only steering.
class PurePursuit {
 public:
  Twist2D compute(const Pose2D& p, Mission& m, float cruise = autonomy::kCruiseSpeed) {
    Twist2D out;
    if (m.done() || m.size() == 0) return out;
    const Waypoint& goal = m.current();
    const float dist_goal = std::hypot(goal.x - p.x, goal.y - p.y);
    if (dist_goal < autonomy::kGoalTolerance) {
      m.advance();
      if (m.done()) return out;
      return compute(p, m, cruise);
    }
    const Waypoint target = carrot(p, m.previous(), goal);
    const float dx = target.x - p.x, dy = target.y - p.y;
    const float c = std::cos(p.yaw), s = std::sin(p.yaw);
    const float xr = c * dx + s * dy;   // target in the rover frame
    const float yr = -s * dx + c * dy;
    const float bearing = std::atan2(yr, xr);
    if (std::fabs(bearing) > 0.9f) {  // ~50 degrees: turn in place
      out.w = (bearing > 0 ? 1.0f : -1.0f) * autonomy::kTurnRate;
      return out;
    }
    const float L2 = xr * xr + yr * yr;
    const float curvature = L2 > 1e-6f ? 2.0f * yr / L2 : 0.0f;
    float v = cruise;
    if (m.last_leg() && dist_goal < 0.4f) v *= clampf(dist_goal / 0.4f, 0.35f, 1.0f);
    out.v = v;
    out.w = clampf(v * curvature, -safety::kMaxTurnRate, safety::kMaxTurnRate);
    return out;
  }

 private:
  static Waypoint carrot(const Pose2D& p, const Waypoint& a, const Waypoint& b) {
    const float ex = b.x - a.x, ey = b.y - a.y;
    const float len2 = ex * ex + ey * ey;
    if (len2 < 1e-6f) return b;
    float t = ((p.x - a.x) * ex + (p.y - a.y) * ey) / len2;
    t = clampf(t, 0.0f, 1.0f);
    const float len = std::sqrt(len2);
    t += autonomy::kLookahead / len;
    if (t >= 1.0f) return b;
    return Waypoint{a.x + t * ex, a.y + t * ey};
  }
};

}  // namespace rover
