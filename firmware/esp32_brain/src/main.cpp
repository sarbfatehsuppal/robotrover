// Rover brain (ESP32): a micro-ROS node on WiFi plus the mission logic.
//
// Two jobs on two cores:
//   core 1 (Arduino loop): talk to the STM32 at 20 Hz and run rover::Brain.
//                          Never waits on WiFi, so AUTO keeps working when
//                          the laptop is out of range or switched off.
//   core 0 (ros task):     WiFi, the micro-ROS agent connection, topics.
//
// ROS 2 interface (all names relative, so a namespace works):
//   sub  cmd_vel          geometry_msgs/Twist         teleop, used in MANUAL
//   sub  rover/climb      std_msgs/Bool               full-torque crawl for teleop
//   sub  rover/mode       std_msgs/String             "hold" | "manual" | "auto" | "restart"
//   sub  rover/waypoints  std_msgs/Float32MultiArray  [mission_id, loop, x0, y0, x1, y1, ...]
//   pub  odom             nav_msgs/Odometry           dead reckoning, frame odom -> base_link
//   pub  rover/range      sensor_msgs/Range           HC-SR04, frame sonar_link
//   pub  rover/status     std_msgs/String             one human-readable line
//
// No laptop? Press the BOOT button to start (or stop) the stored mission.

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/type_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include <cmath>
#include <cstring>

#include "config.hpp"
#include "rover_core/brain.hpp"
#include "rover_core/params.hpp"
#include "rover_core/protocol.hpp"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#error "Copy include/wifi_secrets.example.h to include/wifi_secrets.h and put your WiFi details in it"
#endif

namespace proto = rover::proto;

// ---------------------------------------------------------------- shared state
static rover::Brain g_brain;
static SemaphoreHandle_t g_lock;
static volatile bool g_link_up = false;
static volatile bool g_climb = false;

struct Locked {
  Locked() { xSemaphoreTake(g_lock, portMAX_DELAY); }
  ~Locked() { xSemaphoreGive(g_lock); }
};

// ---------------------------------------------------------------- micro-ROS
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t sub_cmd, sub_climb, sub_mode, sub_wps;
static rcl_publisher_t pub_odom, pub_range, pub_status;

static geometry_msgs__msg__Twist msg_cmd;
static std_msgs__msg__Bool msg_climb;
static std_msgs__msg__String msg_mode;
static std_msgs__msg__Float32MultiArray msg_wps;
static nav_msgs__msg__Odometry msg_odom;
static sensor_msgs__msg__Range msg_range;
static std_msgs__msg__String msg_status;
static char status_buf[256];

static char odom_frame[] = "odom";
static char base_frame[] = "base_link";
static char sonar_frame[] = "sonar_link";

static void set_string(rosidl_runtime_c__String& s, char* text) {
  s.data = text;
  s.size = std::strlen(text);
  s.capacity = s.size + 1;
}

static void on_cmd_vel(const void* m) {
  const auto* t = static_cast<const geometry_msgs__msg__Twist*>(m);
  Locked l;
  g_brain.on_teleop(rover::Twist2D{static_cast<float>(t->linear.x), static_cast<float>(t->angular.z)}, g_climb,
                    millis());
}

static void on_climb(const void* m) { g_climb = static_cast<const std_msgs__msg__Bool*>(m)->data; }

static void on_mode(const void* m) {
  const auto* s = static_cast<const std_msgs__msg__String*>(m);
  char buf[32];
  const size_t n = s->data.size < sizeof buf - 1 ? s->data.size : sizeof buf - 1;
  std::memcpy(buf, s->data.data, n);
  buf[n] = '\0';
  rover::Mode mode = rover::Mode::kHold;
  bool restart;
  if (!rover::parse_mode(buf, mode, restart)) return;
  Locked l;
  if (restart) g_brain.restart_mission();
  g_brain.set_mode(mode, millis());
}

static void on_waypoints(const void* m) {
  const auto* a = static_cast<const std_msgs__msg__Float32MultiArray*>(m);
  if (a->data.size < 4) return;
  const uint32_t id = static_cast<uint32_t>(a->data.data[0]);
  const bool loop = a->data.data[1] > 0.5f;
  rover::Waypoint wps[rover::autonomy::kMaxWaypoints];
  int n = 0;
  for (size_t i = 2; i + 1 < a->data.size && n < rover::autonomy::kMaxWaypoints; i += 2) {
    wps[n++] = rover::Waypoint{a->data.data[i], a->data.data[i + 1]};
  }
  Locked l;
  // The laptop re-sends the mission every couple of seconds so a rover that
  // connects late still gets it. Only load it if it's actually new.
  if (id != g_brain.mission().id()) g_brain.set_waypoints(wps, n, loop, id);
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, cfg::kNodeName, "", &support) != RCL_RET_OK) return false;

  bool ok = true;
  ok &= rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       "cmd_vel") == RCL_RET_OK;
  ok &= rclc_subscription_init_default(&sub_climb, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                       "rover/climb") == RCL_RET_OK;
  ok &= rclc_subscription_init_default(&sub_mode, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                       "rover/mode") == RCL_RET_OK;
  ok &= rclc_subscription_init_default(&sub_wps, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                       "rover/waypoints") == RCL_RET_OK;
  // Odometry is ~700 bytes, more than one UDP packet, so it needs a reliable
  // (fragmenting) publisher. The small ones go best-effort.
  ok &= rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                    "odom") == RCL_RET_OK;
  ok &= rclc_publisher_init_best_effort(&pub_range, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                        "rover/range") == RCL_RET_OK;
  ok &= rclc_publisher_init_best_effort(&pub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                        "rover/status") == RCL_RET_OK;
  if (!ok) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK) return false;
  ok &= rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &on_cmd_vel, ON_NEW_DATA) == RCL_RET_OK;
  ok &= rclc_executor_add_subscription(&executor, &sub_climb, &msg_climb, &on_climb, ON_NEW_DATA) == RCL_RET_OK;
  ok &= rclc_executor_add_subscription(&executor, &sub_mode, &msg_mode, &on_mode, ON_NEW_DATA) == RCL_RET_OK;
  ok &= rclc_executor_add_subscription(&executor, &sub_wps, &msg_wps, &on_waypoints, ON_NEW_DATA) == RCL_RET_OK;
  (void)rmw_uros_sync_session(1000);  // so message stamps line up with the laptop clock
  return ok;
}

static void destroy_entities() {
  rmw_context_t* ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
  (void)rcl_subscription_fini(&sub_cmd, &node);
  (void)rcl_subscription_fini(&sub_climb, &node);
  (void)rcl_subscription_fini(&sub_mode, &node);
  (void)rcl_subscription_fini(&sub_wps, &node);
  (void)rcl_publisher_fini(&pub_odom, &node);
  (void)rcl_publisher_fini(&pub_range, &node);
  (void)rcl_publisher_fini(&pub_status, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

static void prepare_messages() {
  // Incoming messages with variable-size fields need memory set aside up front.
  micro_ros_utilities_memory_conf_t conf = {};
  conf.max_string_capacity = 32;
  conf.max_ros2_type_sequence_capacity = 2;
  conf.max_basic_type_sequence_capacity = 2 + 2 * rover::autonomy::kMaxWaypoints;
  (void)micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), &msg_mode,
                                                  conf);
  (void)micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                                  &msg_wps, conf);

  set_string(msg_odom.header.frame_id, odom_frame);
  set_string(msg_odom.child_frame_id, base_frame);
  // Rough uncertainties: dead reckoning without encoders is not precise.
  msg_odom.pose.covariance[0] = 0.02;
  msg_odom.pose.covariance[7] = 0.02;
  msg_odom.pose.covariance[35] = 0.05;
  msg_odom.twist.covariance[0] = 0.01;
  msg_odom.twist.covariance[7] = 0.01;
  msg_odom.twist.covariance[35] = 0.05;

  set_string(msg_range.header.frame_id, sonar_frame);
  msg_range.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  msg_range.field_of_view = 0.52f;
  msg_range.min_range = 0.02f;
  msg_range.max_range = 4.0f;

  msg_status.data.data = status_buf;
  msg_status.data.capacity = sizeof status_buf;
  msg_status.data.size = 0;
}

static void stamp(builtin_interfaces__msg__Time& t) {
  const int64_t ns = rmw_uros_epoch_nanos();
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

static void publish_all(uint32_t now, uint32_t& last_odom, uint32_t& last_range, uint32_t& last_status) {
  rover::Pose2D pose;
  rover::BodyTwist tw;
  float range;
  bool fresh;
  {
    Locked l;
    pose = g_brain.pose();
    tw = g_brain.twist();
    range = g_brain.range();
    fresh = g_brain.sonar_fresh(now);
    if (now - last_status >= cfg::kStatusPeriodMs) {
      const int n = g_brain.describe(status_buf, sizeof status_buf, now);
      const size_t len = n > 0 ? static_cast<size_t>(n) : 0;
      msg_status.data.size = len < sizeof status_buf ? len : sizeof status_buf - 1;
    }
  }
  if (now - last_odom >= cfg::kOdomPeriodMs) {
    last_odom = now;
    stamp(msg_odom.header.stamp);
    msg_odom.pose.pose.position.x = pose.x;
    msg_odom.pose.pose.position.y = pose.y;
    msg_odom.pose.pose.orientation.z = std::sin(0.5f * pose.yaw);
    msg_odom.pose.pose.orientation.w = std::cos(0.5f * pose.yaw);
    msg_odom.twist.twist.linear.x = tw.vx;
    msg_odom.twist.twist.linear.y = tw.vy;
    msg_odom.twist.twist.angular.z = tw.w;
    (void)rcl_publish(&pub_odom, &msg_odom, nullptr);
  }
  if (now - last_range >= cfg::kRangePeriodMs) {
    last_range = now;
    stamp(msg_range.header.stamp);
    // REP 117: +inf = nothing in range, NaN = no valid reading.
    msg_range.range = !fresh ? NAN : (range > 0.0f ? range : INFINITY);
    (void)rcl_publish(&pub_range, &msg_range, nullptr);
  }
  if (now - last_status >= cfg::kStatusPeriodMs) {
    last_status = now;
    (void)rcl_publish(&pub_status, &msg_status, nullptr);
  }
}

static void ros_task(void*) {
  static char ssid[] = WIFI_SSID;
  static char pass[] = WIFI_PASSWORD;
  IPAddress agent(AGENT_IP);
  set_microros_wifi_transports(ssid, pass, agent, AGENT_PORT);  // blocks until WiFi is up
  prepare_messages();

  enum { kWaiting, kAvailable, kConnected, kLost } state = kWaiting;
  uint32_t last_ping = 0, last_odom = 0, last_range = 0, last_status = 0;
  for (;;) {
    const uint32_t now = millis();
    switch (state) {
      case kWaiting:
        if (now - last_ping > 500) {
          last_ping = now;
          if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = kAvailable;
        }
        break;
      case kAvailable:
        if (create_entities()) {
          state = kConnected;
          g_link_up = true;
        } else {
          destroy_entities();
          state = kWaiting;
        }
        break;
      case kConnected:
        if (now - last_ping > 300) {
          last_ping = now;
          if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            state = kLost;
            break;
          }
        }
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        publish_all(now, last_odom, last_range, last_status);
        break;
      case kLost:
        g_link_up = false;
        destroy_entities();
        state = kWaiting;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ---------------------------------------------------------------- control loop
static proto::Decoder g_decoder;

void setup() {
  Serial.begin(115200);
  Serial2.begin(cfg::kLinkBaud, SERIAL_8N1, cfg::kLinkRxPin, cfg::kLinkTxPin);
  pinMode(cfg::kBootButton, INPUT_PULLUP);
  pinMode(cfg::kLedPin, OUTPUT);
  g_lock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ros_task, "ros", 16384, nullptr, 1, nullptr, 0);
  Serial.println("rover brain up. BOOT button = start/stop mission.");
}

void loop() {
  static uint32_t last_cmd = 0, last_print = 0, pressed_since = 0;
  static bool was_pressed = false;
  const uint32_t now = millis();

  while (Serial2.available() > 0) {
    if (g_decoder.feed(static_cast<uint8_t>(Serial2.read())) && g_decoder.type() == proto::kTelemetry) {
      proto::Telemetry t;
      if (proto::decode(g_decoder.payload(), g_decoder.length(), t)) {
        Locked l;
        g_brain.on_telemetry(t, now);
      }
    }
  }

  // BOOT button: short press toggles AUTO <-> HOLD.
  const bool pressed = digitalRead(cfg::kBootButton) == LOW;
  if (pressed && !was_pressed) pressed_since = now;
  if (!pressed && was_pressed && now - pressed_since > 40) {
    Locked l;
    g_brain.set_mode(g_brain.mode() == rover::Mode::kAuto ? rover::Mode::kHold : rover::Mode::kAuto, now);
  }
  was_pressed = pressed;

  if (now - last_cmd >= cfg::kControlPeriodMs) {
    last_cmd = now;
    proto::DriveCommand cmd;
    rover::Mode mode;
    {
      Locked l;
      g_brain.set_link(g_link_up);
      cmd = g_brain.step(now);
      mode = g_brain.mode();
    }
    uint8_t frame[proto::kMaxFrame];
    Serial2.write(frame, proto::encode(cmd, frame));
    // LED: solid = laptop connected, slow blink = waiting for it, fast = AUTO.
    const bool on = mode == rover::Mode::kAuto ? (now / 120) % 2 : (g_link_up ? true : (now / 600) % 2);
    digitalWrite(cfg::kLedPin, on ? HIGH : LOW);
  }

  if (now - last_print >= 1000) {
    last_print = now;
    char line[200];
    {
      Locked l;
      g_brain.describe(line, sizeof line, now);
    }
    Serial.println(line);
  }
  delay(1);
}
