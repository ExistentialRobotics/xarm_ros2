#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware'
    controllers_name = 'controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    # xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    xarm_type = 'xarm6' #TODO - fix
    ros_namespace = LaunchConfiguration('ros_namespace', default='')

    # # robot driver launch
    # # xarm_api/launch/_robot_driver.launch.py
    # robot_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
    #     launch_arguments={
    #         'robot_ip': robot_ip,
    #         'report_type': report_type,
    #         'dof': dof,
    #         'hw_ns': hw_ns,
    #         'add_gripper': add_gripper,
    #         'prefix': prefix,
    #         'baud_checkset': baud_checkset,
    #         'default_gripper_baud': default_gripper_baud,
    #         'robot_type': robot_type,
    #     }.items(),
    # )
    
    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_description.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'ros2_control_plugin': ros2_control_plugin,
            'joint_states_remapping': PathJoinSubstitution(['/', ros_namespace, hw_ns, 'joint_states']),
        }.items(),
    )

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            # 'add_gripper': add_gripper if robot_type.perform(context) == 'xarm' else 'false',
            'no_gui_ctrl': no_gui_ctrl,
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value,
        }.items(),
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': [f'{prefix}{hw_ns}/joint_states']}],
        remappings=[
            ('follow_joint_trajectory', f'{prefix}{xarm_type}_traj_controller/follow_joint_trajectory'),
        ],
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'ros2_control_plugin': ros2_control_plugin,
        }.items(),
    )

    control_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            f'{prefix}{xarm_type}_traj_controller',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    return LaunchDescription([
        robot_description_launch,
        robot_moveit_common_launch,
        joint_state_publisher_node,
        ros2_control_launch,
        control_node,
        # robot_driver_launch,
    ])


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
