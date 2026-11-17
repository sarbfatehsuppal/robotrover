"""Send the rover its predetermined path.

    ros2 run rover_station mission --ros-args -p mission_file:=/path/to/mission.yaml

Reads a YAML mission, publishes it to the ESP32 as a compact float array on
rover/waypoints, and as a nav_msgs/Path on rover/path so RViz can draw it.
It re-sends every couple of seconds; the rover only reloads when the
mission id changes, so a rover that boots late still picks it up.

Clicking "2D Goal Pose" in RViz replaces the mission with that single point.

Mission file format (metres, in the frame where the rover was powered on,
x forward, y to the left):

    name: hallway and back
    loop: false
    waypoints:
      - [1.0, 0.0]
      - [1.5, 0.6]
"""

from __future__ import annotations

import math
import random
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32MultiArray

MAX_WAYPOINTS = 32  # rover_core autonomy::kMaxWaypoints


class MissionNode(Node):
    def __init__(self):
        super().__init__('rover_mission')
        mission_file = self.declare_parameter('mission_file', '').value
        self.period = float(self.declare_parameter('resend_period', 2.0).value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_wps = self.create_publisher(Float32MultiArray, 'rover/waypoints', 10)
        self.pub_path = self.create_publisher(Path, 'rover/path', latched)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        # Float32 holds integers exactly up to 2^24, keep ids below that.
        self.mission_id = random.randint(1, 1 << 20)
        self.name, self.loop, self.points = 'none', False, []
        if mission_file:
            self.load(mission_file)
        self.create_timer(self.period, self.send)

    def load(self, path: str):
        import yaml

        data = yaml.safe_load(FilePath(path).read_text()) or {}
        pts = [(float(p[0]), float(p[1])) for p in data.get('waypoints', [])]
        if len(pts) > MAX_WAYPOINTS:
            self.get_logger().warn(f'{len(pts)} waypoints, the rover keeps the first {MAX_WAYPOINTS}')
            pts = pts[:MAX_WAYPOINTS]
        self.set_mission(str(data.get('name', FilePath(path).stem)), bool(data.get('loop', False)), pts)

    def set_mission(self, name: str, loop: bool, pts):
        self.name, self.loop, self.points = name, loop, list(pts)
        self.mission_id += 1
        length = sum(math.dist(a, b) for a, b in zip([(0.0, 0.0)] + self.points, self.points))
        self.get_logger().info(f'mission "{name}": {len(pts)} waypoints, {length:.1f} m'
                               f'{", looping" if loop else ""} (id {self.mission_id})')
        self.send()

    def on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        self.set_mission('goal from RViz', False, [(p.x, p.y)])

    def send(self):
        if not self.points:
            return
        arr = Float32MultiArray()
        arr.data = [float(self.mission_id), 1.0 if self.loop else 0.0]
        for x, y in self.points:
            arr.data += [x, y]
        self.pub_wps.publish(arr)

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in [(0.0, 0.0)] + self.points + ([self.points[0]] if self.loop else []):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
