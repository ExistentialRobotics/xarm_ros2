#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    # robot joint state launch
    # xarm_description/launch/_robot_joint_state.launch.py
    robot_joint_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_robot_joint_state.launch.py']),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
        }.items(),
    )

    # rviz2 display launch
    # xarm_description/launch/_rviz_display.launch.py
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_rviz_display.launch.py']),
    )
    
    return LaunchDescription([
        robot_joint_state_launch,
        rviz2_launch
    ])
