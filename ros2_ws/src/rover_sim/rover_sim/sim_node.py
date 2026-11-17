"""ROS 2 wrapper around the plant: the fake physical rover.

Subscribes:
  sim/wheel_cmd     std_msgs/Float32MultiArray  6 motor duties + 2 steering angles (from rover_sil)
Publishes:
  sim/sonar         sensor_msgs/Range           raw HC-SR04 reading (goes into rover_sil's "STM32")
  sim/ground_truth  nav_msgs/Odometry           where the rover really is
  sim/info          std_msgs/String             stuck / bumped / motor torque, for curiosity
  joint_states      sensor_msgs/JointState      rockers, bogies, steering, wheels
  sim/obstacles     visualization_msgs/MarkerArray
  tf                odom -> base_link (true pose, so RViz shows the real rover)
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .params import load_params
from .plant import LEFT, RIGHT, Plant
from .terrain import Box, World

JOINTS = [
    'rocker_left_joint', 'rocker_right_joint', 'bogie_left_joint', 'bogie_right_joint',
    'steer_front_left_joint', 'steer_front_right_joint',
    'wheel_front_left_joint', 'wheel_middle_left_joint', 'wheel_rear_left_joint',
    'wheel_front_right_joint', 'wheel_middle_right_joint', 'wheel_rear_right_joint',
]


def load_world(path: str) -> World:
    import yaml

    data = yaml.safe_load(Path(path).read_text()) or {}
    boxes = []
    for o in data.get('obstacles', []):
        boxes.append(Box(cx=float(o['x']), cy=float(o['y']), length=float(o['length']),
                         width=float(o['width']), height=float(o['height']),
                         yaw=math.radians(float(o.get('yaw_deg', 0.0))), name=str(o.get('name', ''))))
    return World(boxes)


def quaternion(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class SimNode(Node):
    def __init__(self):
        super().__init__('rover_sim')
        world_file = self.declare_parameter('world_file', '').value
        geometry_file = self.declare_parameter('geometry_file', '').value
        self.rate = float(self.declare_parameter('rate', 50.0).value)
        x0 = float(self.declare_parameter('start_x', 0.0).value)
        y0 = float(self.declare_parameter('start_y', 0.0).value)
        yaw0 = math.radians(float(self.declare_parameter('start_yaw_deg', 0.0).value))

        self.params = load_params(geometry_file or None)
        self.world = load_world(world_file) if world_file else World([])
        self.plant = Plant(self.params, self.world, x0, y0, yaw0)
        self.get_logger().info(
            f'simulating a {self.params.total_mass:.2f} kg rover among {len(self.world.boxes)} obstacles')

        self.last_cmd = self.get_clock().now()
        self.create_subscription(Float32MultiArray, 'sim/wheel_cmd', self.on_cmd, 10)
        self.pub_sonar = self.create_publisher(Range, 'sim/sonar', qos_profile_sensor_data)
        self.pub_truth = self.create_publisher(Odometry, 'sim/ground_truth', 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_info = self.create_publisher(String, 'sim/info', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_markers = self.create_publisher(MarkerArray, 'sim/obstacles', latched)
        self.tf = TransformBroadcaster(self)
        self.ticks = 0
        self.last_info = ''
        self.publish_markers()
        self.create_timer(1.0 / self.rate, self.tick)

    def on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) < 8:
            return
        self.plant.command(msg.data[:6], msg.data[6:8])
        self.last_cmd = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds > 5e8:  # nobody driving: brake
            self.plant.command([0.0] * 6, self.plant.steer_cmd)
        st = self.plant.step(1.0 / self.rate)
        self.ticks += 1
        stamp = now.to_msg()

        q = quaternion(st.roll, st.pitch, st.yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = st.x, st.y, st.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)

        js = JointState()
        js.header.stamp = stamp
        js.name = JOINTS
        js.position = [st.rocker_joint[LEFT], st.rocker_joint[RIGHT], st.bogie_joint[LEFT], st.bogie_joint[RIGHT],
                       st.steer[0], st.steer[1]] + list(st.wheel_angle)
        js.velocity = [0.0] * 6 + list(st.wheel_speed)
        self.pub_joints.publish(js)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = st.x, st.y, st.z
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = st.vx, st.vy, st.wz
        self.pub_truth.publish(o)

        if self.ticks % max(1, int(self.rate * 0.06)) == 0:  # HC-SR04 pings every ~60 ms
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = 'sonar_link'
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 2 * self.params.sonar_half_angle
            r.min_range = self.params.sonar_min_range
            r.max_range = self.params.sonar_max_range
            r.range = float(self.plant.sonar())
            self.pub_sonar.publish(r)

        info = []
        if st.stuck:
            info.append(f'stuck: {st.stuck}')
        if st.bumped:
            info.append('bumped into something')
        tau = max(abs(self.plant.sides[LEFT].torque), abs(self.plant.sides[RIGHT].torque))
        if tau > 0.3 * self.params.stall_torque():
            info.append(f'motors working hard: {tau / self.params.stall_torque():.0%} of stall')
        text = '; '.join(info)
        if text != self.last_info or self.ticks % int(self.rate) == 0:
            self.pub_info.publish(String(data=text or 'cruising'))
            self.last_info = text
        if self.ticks % int(2 * self.rate) == 0:
            self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, b in enumerate(self.world.boxes):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = b.cx, b.cy, b.height / 2
            _, _, qz, qw = quaternion(0.0, 0.0, b.yaw)
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
            m.scale.x, m.scale.y, m.scale.z = b.length, b.width, b.height
            climbable = b.height <= 2 * self.params.wheel_radius * 1.15
            m.color.r, m.color.g, m.color.b, m.color.a = (0.80, 0.62, 0.38, 0.9) if climbable else (0.55, 0.30, 0.25, 0.9)
            arr.markers.append(m)
            label = Marker()
            label.header.frame_id = 'odom'
            label.ns = 'labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = b.cx, b.cy
            label.pose.position.z = b.height + 0.05
            label.scale.z = 0.05
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{b.name or "box"} {b.height * 1000:.0f} mm'
            arr.markers.append(label)
        self.pub_markers.publish(arr)


def main():
    rclpy.init()
    node = SimNode()
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
