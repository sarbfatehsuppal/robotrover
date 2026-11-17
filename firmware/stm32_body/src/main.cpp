// Rover body controller (STM32F411 Black Pill).
//
// Listens to the ESP32 over UART, pings the HC-SR04, and every 10 ms turns
// the latest drive command into six motor duties and two steering angles
// via rover::BodyController. Reports back at 20 Hz.
//
// On-board LED:
//   slow blink (1 Hz)    no drive commands, wheels braked
//   solid                obstacle reflex holding the rover
//   quick flicker        happily driving / listening

#include <cmath>

#include "board.hpp"
#include "hw.hpp"
#include "rover_core/body_controller.hpp"
#include "rover_core/protocol.hpp"

using rover::BodyController;
using rover::BodyOutput;
namespace proto = rover::proto;

namespace {

void update_led(uint32_t now, uint8_t status) {
  if (status & proto::kStatusTimeout) {
    hw::led((now / 500) % 2 == 0);
  } else if (status & proto::kStatusReflexStop) {
    hw::led(true);
  } else {
    hw::led((now / 100) % 4 == 0);
  }
}

}  // namespace

int main() {
  hw::init();
  hw::brake_all();
  hw::set_servo(0, 0.0f);
  hw::set_servo(1, 0.0f);

  BodyController body;
  proto::Decoder decoder;
  BodyOutput out;
  uint32_t last_control = hw::millis();
  uint32_t last_report = last_control;
  uint8_t report_seq = 0;

  for (;;) {
    const uint32_t now = hw::millis();

    uint8_t b;
    while (hw::link_read(b)) {
      if (decoder.feed(b) && decoder.type() == proto::kDriveCommand) {
        proto::DriveCommand cmd;
        if (proto::decode(decoder.payload(), decoder.length(), cmd)) body.on_command(cmd, now);
      }
    }

    float range;
    if (hw::sonar_poll(range)) body.on_range(range, now);

    if (now - last_control >= 10) {
      const float dt = static_cast<float>(now - last_control) * 0.001f;
      last_control = now;
      out = body.step(now, dt);
      for (int i = 0; i < 6; ++i) hw::set_motor(i, out.duty[i]);
      hw::set_servo(0, out.steer_target[0]);
      hw::set_servo(1, out.steer_target[1]);
      update_led(now, out.status);
      hw::kick_watchdog();
    }

    if (now - last_report >= 50) {
      last_report = now;
      proto::Telemetry t;
      const float r = body.range();
      t.range_mm = static_cast<uint16_t>(r > 0.0f ? std::fmin(r, 60.0f) * 1000.0f : 0.0f);
      t.v_mm_s = proto::to_i16(out.applied.v, 1000.0f);
      t.w_mrad_s = proto::to_i16(out.applied.w, 1000.0f);
      t.steer_left_mrad = proto::to_i16(out.steer_estimate[0], 1000.0f);
      t.steer_right_mrad = proto::to_i16(out.steer_estimate[1], 1000.0f);
      t.status = out.status;
      t.seq = report_seq++;
      t.uptime_ms = now;
      uint8_t frame[proto::kMaxFrame];
      hw::link_write(frame, proto::encode(t, frame));
    }
  }
}
