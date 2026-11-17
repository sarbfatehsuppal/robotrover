// Numbers the firmware needs at compile time.
//
// Most of these mirror ros2_ws/src/rover_description/config/rover_geometry.yaml
// (marked "yaml"). The microcontrollers can't read YAML, so if you change one
// of those, change it in both places. The rest are tuning values that only
// the firmware uses.
#pragma once

#include <cstdint>

namespace rover {

constexpr float kPi = 3.14159265358979f;

// ---- Wheel layout in the base_link frame -----------------------------------
// base_link sits on the line through the two rocker pivots, halfway between
// them. x forward, y left. Wheel order everywhere: FL, ML, RL, FR, MR, RR.
namespace geo {
constexpr float kWheelRadius = 0.0325f;  // yaml wheel_radius
constexpr float kHalfTrack = 0.115f;     // yaml track_width / 2
constexpr float kFrontX = 0.130f;        // yaml front_axle_x
constexpr float kMiddleX = 0.000f;       // yaml middle_axle_x
constexpr float kRearX = -0.130f;        // yaml rear_axle_x
// Only the front corners steer, so the middle and rear wheels always scrub a
// little in a turn. Putting the turning centre on the line halfway between
// them splits that scrub evenly.
constexpr float kIcrX = 0.5f * (kMiddleX + kRearX);
constexpr float kSteerLimit = 1.05f;  // yaml steer_limit (rad)
constexpr float kSteerRate = 6.0f;    // yaml steer_rate (rad/s)

constexpr int kWheels = 6;
constexpr float kWheelX[kWheels] = {kFrontX, kMiddleX, kRearX, kFrontX, kMiddleX, kRearX};
constexpr float kWheelY[kWheels] = {kHalfTrack, kHalfTrack, kHalfTrack, -kHalfTrack, -kHalfTrack, -kHalfTrack};
constexpr int kFL = 0, kML = 1, kRL = 2, kFR = 3, kMR = 4, kRR = 5;
}  // namespace geo

// ---- TT motor (1:90 metal gearbox) through an MX1616 at 5 V -----------------
namespace motor {
constexpr float kBusVoltage = 5.0f;      // yaml motor_bus_voltage
constexpr float kResistance = 6.5f;      // yaml motor_resistance + driver_resistance
constexpr float kKe = 0.443f;            // yaml motor_ke, V per rad/s at the wheel
constexpr float kNoLoadCurrent = 0.15f;  // yaml motor_no_load_current
// Fastest the wheel turns at 100% duty with nothing pushing back.
constexpr float kMaxWheelSpeed = (kBusVoltage - kNoLoadCurrent * kResistance) / kKe;  // ~9.1 rad/s
constexpr float kMaxSpeed = kMaxWheelSpeed * geo::kWheelRadius;                       // ~0.30 m/s
// Below this duty the gearbox friction wins and the wheel doesn't turn.
constexpr float kStartDuty = kNoLoadCurrent * kResistance / kBusVoltage;  // ~0.20
// Climb mode never drops below this duty while moving. From the climb model
// (python3 -m rover_sim.climb_analysis): the 65 mm step needs at least 85%.
constexpr float kClimbDuty = 0.95f;
}  // namespace motor

// ---- Safety reflex (runs on the STM32) --------------------------------------
namespace safety {
constexpr uint32_t kCommandTimeoutMs = 300;  // no drive command for this long -> stop
constexpr uint32_t kRangeStaleMs = 250;      // ultrasonic reading older than this is ignored
constexpr float kStopDistance = 0.18f;       // m, forward motion refused below this
constexpr float kSlowDistance = 0.40f;       // m, forward speed tapers to zero from here
constexpr float kMaxAccel = 0.8f;            // m/s^2, keeps the PETG joints and gearboxes happy
constexpr float kMaxAngAccel = 4.0f;         // rad/s^2
constexpr float kMaxTurnRate = 2.0f;         // rad/s
}  // namespace safety

// ---- Autonomy (runs on the ESP32, and in the simulator) ---------------------
namespace autonomy {
constexpr float kCruiseSpeed = 0.22f;      // m/s; climb mode turns this into >=95% duty anyway
constexpr float kLookahead = 0.30f;        // m, pure pursuit carrot distance
constexpr float kGoalTolerance = 0.08f;    // m
constexpr float kAvoidDistance = 0.45f;    // m, start an avoidance manoeuvre
constexpr float kClearDistance = 0.90f;    // m, the way counts as clear beyond this
constexpr float kTurnRate = 1.0f;          // rad/s while scanning for a gap
constexpr float kExtraTurn = 0.50f;        // rad (~30 deg) past the edge so the wheels clear it too
constexpr float kBypassDistance = 0.55f;   // m driven straight after finding a gap
constexpr uint32_t kPauseMs = 400;         // think before acting, like the real thing
constexpr uint32_t kTeleopTimeoutMs = 500;
constexpr int kMaxAvoidsPerWaypoint = 4;   // then give up on that waypoint
constexpr int kMaxWaypoints = 32;
}  // namespace autonomy

}  // namespace rover
