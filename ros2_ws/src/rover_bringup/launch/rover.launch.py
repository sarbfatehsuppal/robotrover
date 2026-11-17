"""Laptop side for the real rover.

    ros2 launch rover_bringup rover.launch.py
    ros2 run rover_station teleop           # in a second terminal, to drive

Starts the micro-ROS agent the ESP32 connects to over WiFi (UDP 8888),
the robot model, odom -> TF, the mission uploader and RViz.

Arguments:
    mission:=<file>   predetermined path (config/missions/yard_loop.yaml)
    agent:=false      if you run the agent yourself, e.g. with Docker:
                      docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
    port:=8888
    rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('rover_bringup')
    description = FindPackageShare('rover_description')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([description, 'urdf', 'rover.urdf.xacro'])]), value_type=str)

    args = [
        DeclareLaunchArgument('mission', default_value=PathJoinSubstitution([bringup, 'config', 'missions', 'yard_loop.yaml'])),
        DeclareLaunchArgument('agent', default_value='true'),
        DeclareLaunchArgument('port', default_value='8888'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    return LaunchDescription(args + [
        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent', output='screen',
             arguments=['udp4', '--port', LaunchConfiguration('port')],
             condition=IfCondition(LaunchConfiguration('agent'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # The rover has no sensors on its suspension joints, so they're drawn
        # at rest. Steering and wheels too; this is just for orientation.
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        Node(package='rover_station', executable='odom_tf', name='rover_odom_tf'),
        Node(package='rover_station', executable='mission', name='rover_mission', output='screen',
             parameters=[{'mission_file': LaunchConfiguration('mission')}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([description, 'rviz', 'rover.rviz'])]),
    ])
