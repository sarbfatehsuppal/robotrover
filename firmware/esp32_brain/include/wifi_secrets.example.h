// Copy this file to wifi_secrets.h (which git ignores) and fill it in.
#pragma once

// The network the laptop is on. ESP32s only do 2.4 GHz.
#define WIFI_SSID "your-network"
#define WIFI_PASSWORD "your-password"

// The laptop running `ros2 launch rover_bringup rover.launch.py`
// (which starts the micro-ROS agent on UDP port 8888).
// Find it with `hostname -I` on Linux.
#define AGENT_IP 192, 168, 1, 50
#define AGENT_PORT 8888
