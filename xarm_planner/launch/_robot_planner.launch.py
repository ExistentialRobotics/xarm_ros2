#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import json
from ament_index_python import get_package_share_directory
from launch.frontend import expose
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    node_executable = LaunchConfiguration('node_executable', default='xarm_planner_node')
    node_name = LaunchConfiguration('node_name', default=node_executable)
    node_parameters = LaunchConfiguration('node_parameters', default={})

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = 'xarm6' #TODO: fix later with env var

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
        urdf_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'ros2_control_plugin': ros2_control_plugin,
        },
        srdf_arguments={
            'prefix': prefix,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type,
        }
    )
    kinematics_yaml = robot_description_parameters['robot_description_kinematics']
    joint_limits_yaml = robot_description_parameters.get('robot_description_planning', None)
    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(kinematics_yaml=kinematics_yaml, joint_limits_yaml=joint_limits_yaml, prefix=prefix.perform(context))
    try:
        xarm_planner_parameters = json.loads(node_parameters.perform(context))
    except:
        xarm_planner_parameters = {}

    xarm_planner_node = Node(
        name=node_name,
        package='xarm_planner',
        executable=node_executable,
        output='screen',
        parameters=[
            robot_description_parameters,
            {
                'robot_type': 'xarm', #TODO: fix later
                'dof': '6', #TODO: fix later 
                'prefix': prefix
            },
            xarm_planner_parameters,
        ],
    )

    nodes = [
        xarm_planner_node
    ]
    # if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true') and use_gripper_node.perform(context) in ('True', 'true'):
    #     planning_group = 'uf850_gripper' if robot_type.perform(context) == 'uf850' else 'xarm_gripper'
    #     xarm_gripper_planner_node = Node(
    #         name='xarm_gripper_planner_node',
    #         package='xarm_planner',
    #         executable='xarm_gripper_planner_node',
    #         output='screen',
    #         parameters=[
    #             robot_description_parameters,
    #             {'PLANNING_GROUP': planning_group},
    #         ],
    #     )
    #     nodes.append(xarm_gripper_planner_node)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
