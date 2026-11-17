// Software-in-the-loop: the ESP32 brain and the STM32 body, running on the laptop.
//
// This node runs exactly the classes the two microcontrollers run
// (rover::Brain and rover::BodyController from rover_core), and even passes
// every command and report through the real UART frame encoder/decoder.
// Only the physics is fake: rover_sim's plant node takes our motor duties
// and hands back what the sonar sees.
//
// From the outside it looks like the real rover, same topic names, so the
// keyboard teleop, the mission uploader and RViz can't tell the difference.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_core/body_controller.hpp"
#include "rover_core/brain.hpp"
#include "rover_core/protocol.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
namespace proto = rover::proto;

class SilNode : public rclcpp::Node {
 public:
  SilNode() : Node("rover_sil"), t0_(std::chrono::steady_clock::now()) {
    use_ground_truth_ = declare_parameter("use_ground_truth", false);
    start_mode_ = declare_parameter("start_mode", std::string("hold"));

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          brain_.on_teleop(rover::Twist2D{static_cast<float>(m->linear.x), static_cast<float>(m->angular.z)},
                           climb_, now_ms());
        });
    sub_climb_ = create_subscription<std_msgs::msg::Bool>(
        "rover/climb", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) { climb_ = m->data; });
    sub_mode_ = create_subscription<std_msgs::msg::String>(
        "rover/mode", 10, [this](std_msgs::msg::String::ConstSharedPtr m) { set_mode(m->data); });
    sub_wps_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "rover/waypoints", 10, [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) { on_waypoints(*m); });
    sub_sonar_ = create_subscription<sensor_msgs::msg::Range>(
        "sim/sonar", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Range::ConstSharedPtr m) {
          // Same convention as the STM32 driver: 0 = no echo.
          const float r = std::isfinite(m->range) ? m->range : 0.0f;
          body_.on_range(r, now_ms());
        });
    sub_truth_ = create_subscription<nav_msgs::msg::Odometry>(
        "sim/ground_truth", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          if (!use_ground_truth_) return;
          const auto& q = m->pose.pose.orientation;
          const float yaw = static_cast<float>(std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
          brain_.set_external_pose(rover::Pose2D{static_cast<float>(m->pose.pose.position.x),
                                                 static_cast<float>(m->pose.pose.position.y), yaw});
        });

    pub_wheels_ = create_publisher<std_msgs::msg::Float32MultiArray>("sim/wheel_cmd", 10);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_range_ = create_publisher<sensor_msgs::msg::Range>("rover/range", rclcpp::SensorDataQoS());
    pub_status_ = create_publisher<std_msgs::msg::String>("rover/status", 10);

    brain_.set_link(true);  // the "WiFi" never drops in here
    set_mode(start_mode_);

    body_timer_ = create_wall_timer(10ms, [this] { body_tick(); });
    brain_timer_ = create_wall_timer(50ms, [this] { brain_tick(); });
    pub_timer_ = create_wall_timer(100ms, [this] { publish(); });
    RCLCPP_INFO(get_logger(), "SIL running the rover_core brain + body (ground truth odom: %s)",
                use_ground_truth_ ? "yes" : "no, dead reckoning like the real thing");
  }

 private:
  uint32_t now_ms() const {
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0_).count());
  }

  void set_mode(const std::string& s) {
    rover::Mode m = rover::Mode::kHold;
    bool restart;
    if (!rover::parse_mode(s.c_str(), m, restart)) {
      RCLCPP_WARN(get_logger(), "unknown mode '%s' (try hold, manual, auto, restart)", s.c_str());
      return;
    }
    if (restart) brain_.restart_mission();
    brain_.set_mode(m, now_ms());
    RCLCPP_INFO(get_logger(), "mode -> %s", rover::mode_name(m));
  }

  void on_waypoints(const std_msgs::msg::Float32MultiArray& a) {
    if (a.data.size() < 4) return;
    const auto id = static_cast<uint32_t>(a.data[0]);
    if (id == brain_.mission().id()) return;
    std::vector<rover::Waypoint> w;
    for (size_t i = 2; i + 1 < a.data.size(); i += 2) w.push_back(rover::Waypoint{a.data[i], a.data[i + 1]});
    brain_.set_waypoints(w.data(), static_cast<int>(w.size()), a.data[1] > 0.5f, id);
    RCLCPP_INFO(get_logger(), "mission %u loaded: %zu waypoints", id, w.size());
  }

  // "ESP32": 20 Hz, sends a DriveCommand frame down the "UART".
  void brain_tick() {
    const uint32_t now = now_ms();
    uint8_t frame[proto::kMaxFrame];
    const size_t n = proto::encode(brain_.step(now), frame);
    for (size_t i = 0; i < n; ++i) {
      if (to_body_.feed(frame[i]) && to_body_.type() == proto::kDriveCommand) {
        proto::DriveCommand c;
        if (proto::decode(to_body_.payload(), to_body_.length(), c)) body_.on_command(c, now);
      }
    }
  }

  // "STM32": 100 Hz motor loop, reports back at 20 Hz.
  void body_tick() {
    const uint32_t now = now_ms();
    const float dt = last_body_ ? (now - last_body_) * 0.001f : 0.01f;
    last_body_ = now;
    const rover::BodyOutput out = body_.step(now, dt);

    std_msgs::msg::Float32MultiArray w;
    w.data.assign(out.duty, out.duty + 6);
    w.data.push_back(out.steer_target[0]);
    w.data.push_back(out.steer_target[1]);
    pub_wheels_->publish(w);

    if (++body_count_ % 5 == 0) {
      proto::Telemetry t;
      const float r = body_.range();
      t.range_mm = static_cast<uint16_t>(r > 0.0f ? std::fmin(r, 60.0f) * 1000.0f : 0.0f);
      t.v_mm_s = proto::to_i16(out.applied.v, 1000.0f);
      t.w_mrad_s = proto::to_i16(out.applied.w, 1000.0f);
      t.steer_left_mrad = proto::to_i16(out.steer_estimate[0], 1000.0f);
      t.steer_right_mrad = proto::to_i16(out.steer_estimate[1], 1000.0f);
      t.status = out.status;
      t.seq = static_cast<uint8_t>(body_count_);
      t.uptime_ms = now;
      uint8_t frame[proto::kMaxFrame];
      const size_t n = proto::encode(t, frame);
      for (size_t i = 0; i < n; ++i) {
        if (to_brain_.feed(frame[i]) && to_brain_.type() == proto::kTelemetry) {
          proto::Telemetry got;
          if (proto::decode(to_brain_.payload(), to_brain_.length(), got)) brain_.on_telemetry(got, now);
        }
      }
    }
  }

  void publish() {
    const auto stamp = get_clock()->now();
    const uint32_t now = now_ms();
    const auto& p = brain_.pose();
    const auto& tw = brain_.twist();

    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = p.x;
    o.pose.pose.position.y = p.y;
    o.pose.pose.orientation.z = std::sin(0.5 * p.yaw);
    o.pose.pose.orientation.w = std::cos(0.5 * p.yaw);
    o.twist.twist.linear.x = tw.vx;
    o.twist.twist.linear.y = tw.vy;
    o.twist.twist.angular.z = tw.w;
    pub_odom_->publish(o);

    sensor_msgs::msg::Range r;
    r.header.stamp = stamp;
    r.header.frame_id = "sonar_link";
    r.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    r.field_of_view = 0.52f;
    r.min_range = 0.02f;
    r.max_range = 4.0f;
    const float range = brain_.range();
    r.range = !brain_.sonar_fresh(now) ? NAN : (range > 0.0f ? range : INFINITY);
    pub_range_->publish(r);

    if (++pub_count_ % 5 == 0) {
      char buf[256];
      brain_.describe(buf, sizeof buf, now);
      std_msgs::msg::String s;
      s.data = buf;
      pub_status_->publish(s);
    }
  }

  rover::Brain brain_;
  rover::BodyController body_;
  proto::Decoder to_body_, to_brain_;
  bool climb_ = false;
  bool use_ground_truth_ = false;
  std::string start_mode_;
  std::chrono::steady_clock::time_point t0_;
  uint32_t last_body_ = 0;
  uint32_t body_count_ = 0, pub_count_ = 0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_climb_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wps_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sonar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_truth_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_wheels_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr body_timer_, brain_timer_, pub_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SilNode>());
  rclcpp::shutdown();
  return 0;
}
