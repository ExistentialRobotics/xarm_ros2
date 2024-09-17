#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_1 = LaunchConfiguration('robot_ip_1')
    robot_ip_2 = LaunchConfiguration('robot_ip_2')
    report_type = LaunchConfiguration('report_type', default='normal')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    # robot moveit realmove launch
    # xarm_moveit_config/launch/_dual_robot_moveit_realmove.launch.py
    robot_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip_1': robot_ip_1,
            'robot_ip_2': robot_ip_2,
            'report_type': report_type,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'uf850',
            'no_gui_ctrl': 'false',
            'add_realsense_d435i': add_realsense_d435i,
            'kinematics_suffix': kinematics_suffix,
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_realmove_launch
    ])
