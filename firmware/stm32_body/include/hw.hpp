// Thin hardware layer for the Black Pill. Everything the rover decides is in
// rover_core; this file only knows how to move pins.
#pragma once

#include <cstddef>
#include <cstdint>

namespace hw {

void init();                          // clocks, timers, UART, sonar, watchdog
uint32_t millis();
void kick_watchdog();
void led(bool on);

void set_motor(int index, float duty);  // -1..1, 0 = brake
void brake_all();
void set_servo(int index, float angle_rad);

// UART link to the ESP32. Reading never blocks.
bool link_read(uint8_t& byte);
void link_write(const uint8_t* data, size_t n);

// HC-SR04. Call often; it fires a ping every 60 ms by itself.
// Returns true when a new result is ready: range_m > 0 is a distance,
// range_m == 0 means the ping went out but nothing came back (clear).
// If the sensor is unplugged it simply never returns true.
bool sonar_poll(float& range_m);

}  // namespace hw
