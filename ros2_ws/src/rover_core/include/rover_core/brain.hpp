// The ESP32's job, without any hardware in it.
//
// Three modes:
//   HOLD    wheels stopped, the default after power-up
//   MANUAL  drive commands come from the laptop (keyboard teleop)
//   AUTO    follow the stored path, detour around anything the sonar sees
//
// If the laptop link drops in MANUAL, the rover stops. In AUTO it carries on
// with its mission, the way a Mars rover keeps working when Earth is on the
// other side of the Sun. Flip kStopAutoOnLinkLoss if you'd rather it didn't.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "rover_core/avoidance.hpp"
#include "rover_core/kinematics.hpp"
#include "rover_core/navigation.hpp"
#include "rover_core/params.hpp"
#include "rover_core/protocol.hpp"

namespace rover {

enum class Mode : uint8_t { kHold = 0, kManual = 1, kAuto = 2 };

inline const char* mode_name(Mode m) {
  switch (m) {
    case Mode::kHold: return "hold";
    case Mode::kManual: return "manual";
    case Mode::kAuto: return "auto";
  }
  return "?";
}

// Accepts "hold"/"stop", "manual"/"teleop", "auto"/"autonomous", "restart".
inline bool parse_mode(const char* s, Mode& m, bool& restart) {
  auto eq = [](const char* a, const char* b) {
    while (*a && *b) {
      char ca = (*a >= 'A' && *a <= 'Z') ? static_cast<char>(*a + 32) : *a;
      if (ca != *b) return false;
      ++a;
      ++b;
    }
    return *a == *b;
  };
  restart = false;
  if (eq(s, "hold") || eq(s, "stop")) m = Mode::kHold;
  else if (eq(s, "manual") || eq(s, "teleop")) m = Mode::kManual;
  else if (eq(s, "auto") || eq(s, "autonomous")) m = Mode::kAuto;
  else if (eq(s, "restart")) {
    m = Mode::kAuto;
    restart = true;
  } else return false;
  return true;
}

class Brain {
 public:
  static constexpr bool kStopAutoOnLinkLoss = false;

  Brain() {
    // A small default loop so AUTO does something even with no laptop:
    // a 1.2 m x 0.8 m rectangle starting where the rover was switched on.
    const Waypoint square[] = {{1.2f, 0.0f}, {1.2f, 0.8f}, {0.0f, 0.8f}, {0.0f, 0.0f}};
    mission_.set(square, 4, false, 0, Pose2D{});
  }

  void set_mode(Mode m, uint32_t now) {
    if (m == Mode::kAuto && mode_ != Mode::kAuto) {
      avoid_.reset();
      if (mission_.done()) mission_.restart(odom_.pose());
      leg_index_ = -1;  // fresh time budget for the current leg
    }
    mode_ = m;
    mode_since_ = now;
  }
  void restart_mission() {
    mission_.restart(odom_.pose());
    avoid_.reset();
    leg_index_ = -1;
  }
  Mode mode() const { return mode_; }

  void on_teleop(const Twist2D& t, bool climb, uint32_t now) {
    teleop_ = t;
    teleop_climb_ = climb;
    last_teleop_ = now;
    have_teleop_ = true;
  }

  // Waypoints are in the odometry frame (where the rover was switched on).
  void set_waypoints(const Waypoint* w, int n, bool loop, uint32_t id) {
    mission_.set(w, n, loop, id, odom_.pose());
    avoid_.reset();
    leg_index_ = -1;
  }
  const Mission& mission() const { return mission_; }

  // Called whenever the STM32 reports in (about 20 Hz).
  void on_telemetry(const proto::Telemetry& t, uint32_t now) {
    const float dt = have_telem_ ? static_cast<float>(now - last_telem_) * 0.001f : 0.0f;
    last_telem_ = now;
    have_telem_ = true;
    if (!use_external_pose_) odom_.integrate(Twist2D{t.v_mm_s * 0.001f, t.w_mrad_s * 0.001f}, dt);
    range_ = t.range_mm ? t.range_mm * 0.001f : 0.0f;
    sonar_fresh_ = (t.status & proto::kStatusRangeValid) != 0;
    body_status_ = t.status;
    if (sonar_fresh_) hazards_.observe(odom_.pose(), range_, now);
  }

  // Simulator only: use the true pose instead of dead reckoning.
  void set_external_pose(const Pose2D& p) {
    use_external_pose_ = true;
    odom_.set(p);
  }
  void reset_odometry(const Pose2D& p = Pose2D{}) { odom_.reset(p); }

  void set_link(bool up) { link_up_ = up; }
  bool link_up() const { return link_up_; }

  const Pose2D& pose() const { return odom_.pose(); }
  const BodyTwist& twist() const { return odom_.twist(); }
  float range() const { return range_; }
  bool sonar_fresh(uint32_t now) const { return sonar_fresh_ && body_alive(now); }
  uint8_t body_status() const { return body_status_; }
  bool body_alive(uint32_t now) const { return have_telem_ && (now - last_telem_) < 500; }
  const Avoidance& avoidance() const { return avoid_; }

  proto::DriveCommand step(uint32_t now) {
    Twist2D cmd;
    uint8_t flags = 0;
    Mode effective = mode_;
    if (!link_up_ && (mode_ == Mode::kManual || kStopAutoOnLinkLoss)) effective = Mode::kHold;

    if (effective == Mode::kManual) {
      if (have_teleop_ && (now - last_teleop_) < autonomy::kTeleopTimeoutMs) {
        cmd = teleop_;
        if (teleop_climb_) flags |= proto::kFlagClimb;
      }
    } else if (effective == Mode::kAuto && !mission_.done()) {
      const int before = mission_.index();
      const bool fresh = sonar_fresh(now);
      // Which side is the goal on? Turn that way first when dodging.
      const Waypoint& g = mission_.current();
      const float bearing = wrap_pi(std::atan2(g.y - pose().y, g.x - pose().x) - pose().yaw);
      const float goal_side = bearing >= 0 ? 1.0f : -1.0f;
      if (leg_index_ != mission_.index() || leg_mission_ != mission_.id()) start_leg(now);

      // No encoders means no way to feel the rover is stuck. So each leg
      // gets a time budget; overrunning it counts as hitting something.
      if (!avoid_.active() && now - leg_start_ > leg_budget_ms_) {
        avoid_.force(pose(), goal_side, now);
        start_leg(now);
      }
      const bool was_avoiding = avoid_.active();
      if (avoid_.state() == Avoidance::State::kBlocked) {
        mission_.advance();  // give up on this waypoint, try the next one
        avoid_.reset();
        ++skipped_;
      } else if (!avoid_.update(range_, fresh, pose(), goal_side, now, cmd, &hazards_)) {
        if (was_avoiding) {
          mission_.skip_passed(pose());
          mission_.rebase(pose());
          start_leg(now);
        }
        cmd = pursuit_.compute(pose(), mission_);
      }
      if (mission_.index() != before) avoid_.new_waypoint();
      flags |= proto::kFlagClimb;  // Mars rule: you never know what the next rock is like
    }

    proto::DriveCommand out;
    out.v_mm_s = proto::to_i16(cmd.v, 1000.0f);
    out.w_mrad_s = proto::to_i16(cmd.w, 1000.0f);
    out.flags = flags;
    out.seq = seq_++;
    last_cmd_ = cmd;
    return out;
  }

  const Twist2D& last_command() const { return last_cmd_; }
  int skipped_waypoints() const { return skipped_; }

  // One-line, human-readable status for /rover/status.
  int describe(char* buf, int n, uint32_t now) const {
    return std::snprintf(
        buf, static_cast<size_t>(n),
        "mode=%s link=%s body=%s wp=%d/%d%s skipped=%d avoid=%s range=%.2f x=%.2f y=%.2f yaw=%.0f%s%s%s",
        mode_name(mode_), link_up_ ? "up" : "down", body_alive(now) ? "ok" : "silent",
        mission_.size() ? mission_.index() + 1 : 0, mission_.size(), mission_.done() ? "(done)" : "", skipped_,
        Avoidance::name(avoid_.state()), static_cast<double>(range_), static_cast<double>(pose().x),
        static_cast<double>(pose().y), static_cast<double>(pose().yaw * 57.2958f),
        (body_status_ & proto::kStatusReflexStop) ? " REFLEX_STOP" : "",
        (body_status_ & proto::kStatusTimeout) ? " BODY_TIMEOUT" : "",
        (body_status_ & proto::kStatusClimb) ? " climb" : "");
  }

 private:
  void start_leg(uint32_t now) {
    leg_index_ = mission_.index();
    leg_mission_ = mission_.id();
    leg_start_ = now;
    const Waypoint& g = mission_.current();
    const float dist = std::hypot(g.x - pose().x, g.y - pose().y);
    leg_budget_ms_ = static_cast<uint32_t>(1000.0f * (2.0f * dist / autonomy::kCruiseSpeed + 6.0f));
  }

  Mode mode_ = Mode::kHold;
  uint32_t mode_since_ = 0;
  int leg_index_ = -1;
  uint32_t leg_mission_ = 0, leg_start_ = 0, leg_budget_ms_ = 0;
  int skipped_ = 0;
  Twist2D teleop_;
  bool teleop_climb_ = false, have_teleop_ = false;
  uint32_t last_teleop_ = 0;
  Mission mission_;
  PurePursuit pursuit_;
  Avoidance avoid_;
  HazardMemory hazards_;
  Odometry odom_;
  bool use_external_pose_ = false;
  bool have_telem_ = false;
  uint32_t last_telem_ = 0;
  float range_ = 0.0f;
  bool sonar_fresh_ = false;
  uint8_t body_status_ = 0;
  bool link_up_ = false;
  uint8_t seq_ = 0;
  Twist2D last_cmd_;
};

}  // namespace rover
