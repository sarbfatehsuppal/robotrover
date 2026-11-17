"""Turn the rover's /odom into the odom -> base_link transform for RViz.

The ESP32 publishes odometry but not TF (TF over micro-ROS costs more
memory than it's worth), so this does it on the laptop. Not needed in the
simulator, where rover_sim publishes the true pose instead.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__('rover_odom_tf')
        self.height = float(self.declare_parameter('base_height', 0.1075).value)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

    def on_odom(self, o: Odometry):
        t = TransformStamped()
        t.header = o.header
        t.child_frame_id = o.child_frame_id or 'base_link'
        t.transform.translation.x = o.pose.pose.position.x
        t.transform.translation.y = o.pose.pose.position.y
        t.transform.translation.z = self.height  # no IMU, so assume level ground
        t.transform.rotation = o.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
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
