// Wire format between the ESP32 (brain) and the STM32 (body), over UART.
//
//   0xA5 0x5A | type | length | payload ... | crc16 lo | crc16 hi
//
// CRC-16/CCITT-FALSE over type, length and payload. Numbers are little
// endian, speeds in mm/s and mrad/s so everything fits in an int16.
// The ESP32 sends a DriveCommand at 20 Hz; that stream is also the
// heartbeat, so if it stops the STM32 stops the wheels.
#pragma once

#include <cstddef>
#include <cstdint>

namespace rover {
namespace proto {

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kMaxPayload = 32;
constexpr size_t kMaxFrame = 4 + kMaxPayload + 2;

enum Type : uint8_t {
  kDriveCommand = 0x01,  // ESP32 -> STM32
  kTelemetry = 0x81,     // STM32 -> ESP32
};

enum DriveFlags : uint8_t {
  kFlagClimb = 1 << 0,    // full-torque crawl, see motor::kClimbDuty
  kFlagNoReflex = 1 << 1, // ignore the ultrasonic stop (bench testing only)
};

enum StatusFlags : uint8_t {
  kStatusTimeout = 1 << 0,      // no drive command recently, wheels stopped
  kStatusReflexStop = 1 << 1,   // something close in front, forward motion refused
  kStatusReflexSlow = 1 << 2,   // something ahead, forward speed limited
  kStatusClimb = 1 << 3,        // climb mode active
  kStatusRangeValid = 1 << 4,   // the sonar is alive and range_mm is fresh
  kStatusSteering = 1 << 5,     // waiting for the steering servos to get there
};

struct DriveCommand {
  int16_t v_mm_s = 0;
  int16_t w_mrad_s = 0;
  uint8_t flags = 0;
  uint8_t seq = 0;
  static constexpr uint8_t kSize = 6;
};

struct Telemetry {
  uint16_t range_mm = 0;  // 0 = no echo, nothing within ~4 m
  int16_t v_mm_s = 0;     // what the body is actually commanding the wheels to do
  int16_t w_mrad_s = 0;
  int16_t steer_left_mrad = 0;
  int16_t steer_right_mrad = 0;
  uint8_t status = 0;
  uint8_t seq = 0;
  uint32_t uptime_ms = 0;
  static constexpr uint8_t kSize = 16;
};

inline uint16_t crc16(const uint8_t* data, size_t n, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put_u32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
}
inline uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline int16_t to_i16(float x, float scale) {
  float v = x * scale;
  if (v > 32767.0f) v = 32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return static_cast<int16_t>(v >= 0 ? v + 0.5f : v - 0.5f);
}

// Wraps a payload into a frame. Returns the frame length.
inline size_t encode(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out) {
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = type;
  out[3] = len;
  for (uint8_t i = 0; i < len; ++i) out[4 + i] = payload[i];
  const uint16_t c = crc16(out + 2, 2u + len);
  put_u16(out + 4 + len, c);
  return 6u + len;
}

inline size_t encode(const DriveCommand& m, uint8_t* out) {
  uint8_t p[DriveCommand::kSize];
  put_u16(p, static_cast<uint16_t>(m.v_mm_s));
  put_u16(p + 2, static_cast<uint16_t>(m.w_mrad_s));
  p[4] = m.flags;
  p[5] = m.seq;
  return encode(kDriveCommand, p, sizeof p, out);
}

inline size_t encode(const Telemetry& m, uint8_t* out) {
  uint8_t p[Telemetry::kSize];
  put_u16(p, m.range_mm);
  put_u16(p + 2, static_cast<uint16_t>(m.v_mm_s));
  put_u16(p + 4, static_cast<uint16_t>(m.w_mrad_s));
  put_u16(p + 6, static_cast<uint16_t>(m.steer_left_mrad));
  put_u16(p + 8, static_cast<uint16_t>(m.steer_right_mrad));
  p[10] = m.status;
  p[11] = m.seq;
  put_u32(p + 12, m.uptime_ms);
  return encode(kTelemetry, p, sizeof p, out);
}

inline bool decode(const uint8_t* p, uint8_t len, DriveCommand& m) {
  if (len != DriveCommand::kSize) return false;
  m.v_mm_s = static_cast<int16_t>(get_u16(p));
  m.w_mrad_s = static_cast<int16_t>(get_u16(p + 2));
  m.flags = p[4];
  m.seq = p[5];
  return true;
}

inline bool decode(const uint8_t* p, uint8_t len, Telemetry& m) {
  if (len != Telemetry::kSize) return false;
  m.range_mm = get_u16(p);
  m.v_mm_s = static_cast<int16_t>(get_u16(p + 2));
  m.w_mrad_s = static_cast<int16_t>(get_u16(p + 4));
  m.steer_left_mrad = static_cast<int16_t>(get_u16(p + 6));
  m.steer_right_mrad = static_cast<int16_t>(get_u16(p + 8));
  m.status = p[10];
  m.seq = p[11];
  m.uptime_ms = get_u32(p + 12);
  return true;
}

// Byte-at-a-time frame parser. Safe to feed from a UART interrupt.
class Decoder {
 public:
  // Returns true when a complete, CRC-checked frame has arrived.
  bool feed(uint8_t b) {
    switch (state_) {
      case kWaitSync0:
        if (b == kSync0) state_ = kWaitSync1;
        break;
      case kWaitSync1:
        state_ = (b == kSync1) ? kType : (b == kSync0 ? kWaitSync1 : kWaitSync0);
        break;
      case kType:
        type_ = b;
        state_ = kLen;
        break;
      case kLen:
        if (b > kMaxPayload) {
          state_ = kWaitSync0;
          ++errors_;
          break;
        }
        len_ = b;
        idx_ = 0;
        state_ = len_ ? kPayload : kCrc0;
        break;
      case kPayload:
        buf_[idx_++] = b;
        if (idx_ >= len_) state_ = kCrc0;
        break;
      case kCrc0:
        crc_lo_ = b;
        state_ = kCrc1;
        break;
      case kCrc1: {
        state_ = kWaitSync0;
        uint8_t head[2] = {type_, len_};
        uint16_t c = crc16(head, 2);
        c = crc16(buf_, len_, c);
        if (c == static_cast<uint16_t>(crc_lo_ | (b << 8))) return true;
        ++errors_;
        break;
      }
    }
    return false;
  }

  uint8_t type() const { return type_; }
  uint8_t length() const { return len_; }
  const uint8_t* payload() const { return buf_; }
  uint32_t errors() const { return errors_; }

 private:
  enum State : uint8_t { kWaitSync0, kWaitSync1, kType, kLen, kPayload, kCrc0, kCrc1 };
  State state_ = kWaitSync0;
  uint8_t type_ = 0, len_ = 0, idx_ = 0, crc_lo_ = 0;
  uint8_t buf_[kMaxPayload] = {};
  uint32_t errors_ = 0;
};

}  // namespace proto
}  // namespace rover
