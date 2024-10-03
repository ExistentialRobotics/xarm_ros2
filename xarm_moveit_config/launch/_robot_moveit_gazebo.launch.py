#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros2_control_plugin = 'ign_ros2_control/IgnitionSystem'
    controllers_name = 'controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    # robot moveit common launch
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'no_gui_ctrl': no_gui_ctrl,
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value,
            'use_sim_time': 'true'
        }.items(),
    )

    # robot gazebo launch
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'launch', '_robot_beside_table_gazebo.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'ros2_control_plugin': ros2_control_plugin,
            'load_controller': 'true',
        }.items(),
    )

    return LaunchDescription([
        robot_gazebo_launch,
        robot_moveit_common_launch,
    ])