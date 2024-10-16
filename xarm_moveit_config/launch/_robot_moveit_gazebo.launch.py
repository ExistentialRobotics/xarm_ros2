#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    camera_namespace = LaunchConfiguration('camera_namespace', default='camera_01')
    controllers_name = LaunchConfiguration('controllers_name', default='controllers')
    hw_ns = LaunchConfiguration('hw_ns', default='')
    load_controller = LaunchConfiguration('load_controller', default='true')
    moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_simple_controller_manager') 
    moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_simple_controller_manager/MoveItSimpleControllerManager') 
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    prefix = LaunchConfiguration('prefix', default='')
    robot_type= LaunchConfiguration('robot_type', default='xarm6')
    ros2_control_plugin =  LaunchConfiguration('ros2_control_plugin', default='ign_ros2_control/IgnitionSystem') 
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # robot moveit common launch
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common.launch.py'])),
        launch_arguments={
            'controllers_name': controllers_name,
            'hw_ns': hw_ns,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value,
            'no_gui_ctrl': no_gui_ctrl,
            'prefix': prefix,
            'robot_type': robot_type,
            'ros2_control_plugin': ros2_control_plugin,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # robot gazebo launch
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'launch', '_robot_beside_table_gazebo.launch.py'])),
        launch_arguments={
            'camera_namespace': camera_namespace,
            'load_controller': load_controller,
            'no_gui_ctrl': no_gui_ctrl,
            'prefix': prefix,
            'ros2_control_plugin': ros2_control_plugin,
        }.items(),
    )

    return LaunchDescription([
        robot_gazebo_launch,
        robot_moveit_common_launch,
    ])

# ros2 run tf2_ros static_transform_publisher --x 0.5 --y 0.9 --z 1.15 --roll 0 --pitch -1.57 --yaw -0.33 --frame-id world --child-frame-id camera_01_bottom_screw_frame
# ros2 run tf2_ros static_transform_publisher --x 0 --y -0.3 --z 1.021 --roll 0 --pitch 0 --yaw 1.571 --frame-id world --child-frame-id xarm6_link_base




