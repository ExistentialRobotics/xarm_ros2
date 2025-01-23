#!/usr/bin/env python3
# Software License Agreement (BSD License)

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.uf_robot_utils import get_xacro_command

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    kinematics_suffix = LaunchConfiguration('hw_ns', default='')

    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']))

    xarm_type = 'xarm6' #TODO - fix

    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', f'{xarm_type}_controllers.yaml'),
        prefix=prefix.perform(context), 
        ros_namespace=LaunchConfiguration('ros_namespace', default='').perform(context),
    )

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    robot_description = {
        'robot_description': get_xacro_command(
            xacro_file=xacro_file, 
            mappings={
                'prefix': prefix,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'kinematics_suffix': kinematics_suffix, 
            }
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        # BL: don't remap /tf to custom topics. 
        # I.e., multiple robots publish to same /tf topic 
        # Instead, use prefix to distinguish different robots
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static'),
        # ]
        # namespace=this_robot_namespace
    )


    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py'))
    generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), node_name='ufactory_driver'
    )

    # ros2 control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_control_params,
            robot_params,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output='screen',
    )

    return [
        ros2_control_node,
        robot_state_publisher_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


