#pragma once

#include <cstdint>

namespace cfg {

// UART2 to the STM32 (cross over: ESP32 TX -> STM32 RX).
constexpr int kLinkRxPin = 16;  // from STM32 PA9
constexpr int kLinkTxPin = 17;  // to STM32 PA10
constexpr uint32_t kLinkBaud = 115200;

constexpr int kBootButton = 0;  // press to start / stop the stored mission without a laptop
constexpr int kLedPin = 2;      // on-board LED on most ESP32 DevKit boards

constexpr uint32_t kControlPeriodMs = 50;  // 20 Hz to the STM32
constexpr uint32_t kOdomPeriodMs = 100;
constexpr uint32_t kRangePeriodMs = 100;
constexpr uint32_t kStatusPeriodMs = 500;

constexpr const char* kNodeName = "rover_esp32";

}  // namespace cfg
