"""Drive the rover from the laptop keyboard.

    ros2 run rover_station teleop

Works the same against the simulator and the real rover. Unlike
teleop_twist_keyboard it keeps publishing your current command at 10 Hz,
so the rover doesn't stutter waiting for key repeats, and it has keys for
the rover's modes.
"""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

HELP = r"""
  Rover teleop
  ------------------------------------------------------------
     w            faster forward          m   manual mode (keyboard drives)
  a  s  d         turn left / slower /    g   go: autonomous mission
                  turn right              h   hold (stop everything)
     space        stop                    r   restart the mission from here
                                          c   climb mode on/off (full torque)
  q / Ctrl-C      quit (sends stop + hold)
  ------------------------------------------------------------
  Tip: rocks and steps need climb mode, slow speeds don't have the torque.
"""

V_STEP, W_STEP = 0.04, 0.25
V_MAX, W_MAX = 0.30, 2.0


class Teleop(Node):
    def __init__(self):
        super().__init__('rover_teleop')
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, 'rover/mode', 10)
        self.pub_climb = self.create_publisher(Bool, 'rover/climb', 10)
        # The ESP32 publishes status best-effort; a best-effort subscriber hears both it and the simulator.
        self.create_subscription(String, 'rover/status', self.on_status, qos_profile_sensor_data)
        self.v = 0.0
        self.w = 0.0
        self.climb = False
        self.status = 'waiting for /rover/status ...'
        self.create_timer(0.1, self.publish)

    def on_status(self, msg: String):
        self.status = msg.data

    def publish(self):
        t = Twist()
        t.linear.x = self.v
        t.angular.z = self.w
        self.pub_cmd.publish(t)
        self.pub_climb.publish(Bool(data=self.climb))

    def mode(self, name: str):
        self.pub_mode.publish(String(data=name))

    def key(self, k: str) -> bool:
        if k == 'w':
            self.v = min(V_MAX, self.v + V_STEP)
        elif k == 's':
            self.v = max(-V_MAX, self.v - V_STEP)
        elif k == 'a':
            self.w = min(W_MAX, self.w + W_STEP)
        elif k == 'd':
            self.w = max(-W_MAX, self.w - W_STEP)
        elif k == ' ':
            self.v = self.w = 0.0
        elif k == 'c':
            self.climb = not self.climb
        elif k == 'm':
            self.mode('manual')
        elif k == 'g':
            self.v = self.w = 0.0
            self.mode('auto')
        elif k == 'h':
            self.v = self.w = 0.0
            self.mode('hold')
        elif k == 'r':
            self.mode('restart')
        elif k in ('q', '\x03'):
            return False
        return True


def read_key(timeout: float) -> str:
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else ''


def main():
    if not sys.stdin.isatty():
        print('teleop needs a real terminal (run it with `ros2 run`, not inside a launch file)')
        return
    rclpy.init()
    node = Teleop()
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            k = read_key(0.05)
            if k and not node.key(k.lower() if k != '\x03' else k):
                break
            line = (f'v={node.v:+.2f} m/s  w={node.w:+.2f} rad/s  climb={"ON " if node.climb else "off"} | '
                    f'{node.status}')
            sys.stdout.write('\r' + line[:160].ljust(160))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.v = node.w = 0.0
        node.publish()
        node.mode('hold')
        print('\nstopped, rover on hold. bye!')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
