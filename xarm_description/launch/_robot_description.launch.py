#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.uf_robot_utils import get_xacro_command

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    joint_states_remapping = LaunchConfiguration('joint_states_remapping', default='joint_states')
    xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']))

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    robot_description = {
        'robot_description': get_xacro_command(
            xacro_file=xacro_file, 
            mappings={
                'prefix': prefix,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'add_bio_gripper': add_bio_gripper,
                'dof': dof,
                'robot_type': robot_type,
                'ros2_control_plugin': ros2_control_plugin,
                'add_realsense_d435i': add_realsense_d435i,
                'add_d435i_links': add_d435i_links,
                'model1300': model1300,
                'robot_sn': robot_sn,
                'attach_to': attach_to,
                'attach_xyz': attach_xyz,
                'attach_rpy': attach_rpy,
                'kinematics_suffix': kinematics_suffix,
            }
        )
    }

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    return [
        robot_state_publisher_node
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
