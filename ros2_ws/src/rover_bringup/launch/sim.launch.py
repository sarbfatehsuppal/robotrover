"""The whole rover in simulation: physics, firmware logic, mission, RViz.

    ros2 launch rover_bringup sim.launch.py
    ros2 run rover_station teleop           # in a second terminal, to drive

Arguments:
    world:=<file>          obstacle layout (config/worlds/mars_yard.yaml)
    mission:=<file>        predetermined path (config/missions/yard_loop.yaml)
    ground_truth:=true     give the brain perfect odometry instead of dead reckoning
    start_mode:=manual     hold | manual | auto
    rviz:=false            headless
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
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([bringup, 'config', 'worlds', 'mars_yard.yaml'])),
        DeclareLaunchArgument('mission', default_value=PathJoinSubstitution([bringup, 'config', 'missions', 'yard_loop.yaml'])),
        DeclareLaunchArgument('ground_truth', default_value='false'),
        DeclareLaunchArgument('start_mode', default_value='manual'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    return LaunchDescription(args + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='rover_sim', executable='sim_node', name='rover_sim', output='screen',
             parameters=[{'world_file': LaunchConfiguration('world')}]),
        Node(package='rover_sil', executable='sil_node', name='rover_sil', output='screen',
             parameters=[{'use_ground_truth': ParameterValue(LaunchConfiguration('ground_truth'), value_type=bool),
                          'start_mode': LaunchConfiguration('start_mode')}]),
        Node(package='rover_station', executable='mission', name='rover_mission', output='screen',
             parameters=[{'mission_file': LaunchConfiguration('mission')}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([description, 'rviz', 'rover.rviz'])]),
    ])
