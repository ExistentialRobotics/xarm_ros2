#!/usr/bin/env python3

import os
import xacro
import math
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from uf_ros_lib.uf_robot_utils import get_xacro_command

import yaml 

def build_robot_description(this_robot_prefix="", this_robot_namespace="", add_gripper = False):
    # ros2 control params
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='ign_ros2_control/IgnitionSystem')
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'),'config', 'xarm6_controllers.yaml'),
        prefix=this_robot_prefix, 
        add_gripper= add_gripper,#TODO: fix this later
        add_bio_gripper=False,
        ros_namespace=this_robot_namespace,
        update_rate=1000,
        robot_type='xarm'
    )

    # Export the generated ros2 control params
    with open(f'ros2_control_params.yaml', 'w') as file:
        yaml.dump(ros2_control_params, file, sort_keys=False)
    print(f"Generated temporary control params at {ros2_control_params}")

    # robot_description
    robot_description = {
        'robot_description': get_xacro_command(
            xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']), 
            mappings={
                'prefix': this_robot_prefix,
                'hw_ns': this_robot_namespace,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'robot_spec_config_file': PathJoinSubstitution([FindPackageShare('xarm_description'), 'config', 'default_urdf_arguments', 'xarm6.yaml']),
            }
        ),
    }
    return robot_description

def build_camera_description(camera_namespace=""):
    camera_robot_description = {
        'robot_description': get_xacro_command(
            # xacro_file=PathJoinSubstitution([FindPackageShare('main'), 'urdf', 'camera', 'sensor_d455.urdf.xacro']), 
            xacro_file=PathJoinSubstitution([FindPackageShare('realsense2_description'), 'urdf', 'test_d455_camera.urdf.xacro']), 
            mappings={
                'prefix': camera_namespace,
            }
        ),
    }
    return camera_robot_description

def get_per_robot_stack(robot_idx, load_controller):
    """
    The following happens for each robot
    """
    this_robot_namespace = f""
    this_robot_prefix = f"xarm6_"
    add_gripper = False
    nodes_to_launch = []

    robot_description = build_robot_description(this_robot_prefix=this_robot_prefix, this_robot_namespace=this_robot_namespace,add_gripper = add_gripper)

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        # BL: don't remap /tf to custom topics. 
        # I.e., multiple robots publish to same /tf topic 
        # Instead, use prefix to distinguish different robots
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static'),
        # ]
        namespace=this_robot_namespace
    )

    nodes_to_launch.append(
        robot_state_publisher_node            
    )

    spawn_entity_test_node = Node(
        package="keti_gz_utils",
        executable="create_on_table",
        namespace=this_robot_namespace,
        output='screen',
        # NOTE: this version uses parameters instead of CLI arguments.
        # This leads to cleaner dependencies.
        parameters=[{
            'use_sim_time': True,
            'topic': 'robot_description',
            'allow_renaming': False,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'Y': 0.0
        }],
    )

    nodes_to_launch.append(spawn_entity_test_node)

    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        # the below becomes something like xarm0_xarm6_traj_controller. 
        # The first "xarm0" is from prefix. 
        # The second needs to match what's in xarm_control/config/*.yaml 
        f'{this_robot_prefix}traj_controller',
    ]

    if load_controller:
        load_controllers = [
            Node(
                package='controller_manager',
                executable="spawner",
                output='screen',
                arguments=[
                    controller,
                    # spawner is already in the namespace of ros_namespace
                    # so just refer to controller_manager relatively 
                    # (since gazebo_ros2_control is in correct ros_namespace)
                    # note the lack of /
                    '--controller-manager', f'{this_robot_namespace}/controller_manager',
                    '--namespace', f'{this_robot_namespace}'
                ],
                parameters=[{'use_sim_time': True}],
            )
            for controller in controllers
        ]

        nodes_to_launch.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity_test_node,
                    on_exit=load_controllers
                )
            )
        )
        
    return nodes_to_launch

def generate_launch_description():
    camera_namespace = LaunchConfiguration('camera_namespace', default='camera_01')
    camera_robot_description = build_camera_description(camera_namespace=camera_namespace)
    load_controller_config = LaunchConfiguration('load_controller', default=True)
    num_robots_config = LaunchConfiguration('num_robots', default=1)
    world_path = os.path.join(get_package_share_directory('xarm_gazebo'), 'worlds', 'table_world.sdf')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': f' -r {world_path}',
        }.items(),
    )

    # Node for creating bridge between gazebo and ros2 (ros2 run ros_gz_bridge parameter_bridge)
    parameters_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            [camera_namespace, '/camera_color@sensor_msgs/msg/Image@ignition.msgs.Image'],
            [camera_namespace, '/camera_depth@sensor_msgs/msg/Image@ignition.msgs.Image'],
            [camera_namespace, '/camera_depth/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
            [camera_namespace, '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
            [camera_namespace, '/camera_ired1@sensor_msgs/msg/Image@ignition.msgs.Image'],
            [camera_namespace, '/camera_ired2@sensor_msgs/msg/Image@ignition.msgs.Image'],
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/model/realsense2_camera/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/xarm_device/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/block/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            # Note: goal and table_box poses are handled by env_publisher.py for static reference
        ]
    )

    relay_nodes = [
        Node(
            package="topic_tools",
            executable="relay",
            arguments=[
                f"/model/{source}/pose",
                "/tf"
            ]
        ) for source in ["realsense2_camera", "xarm_device", "block"]
        # Note: goal and table_box relay handled by env_publisher.py for static reference poses
    ]

    # Delay parameter_bridge to ensure Gazebo models are fully loaded
    delayed_parameter_bridge = TimerAction(
        period=3.0,  # Delay 3 seconds to ensure Gazebo models are loaded
        actions=[parameters_bridge] + relay_nodes
    )

    nodes_to_launch = [
        gazebo_launch,
        delayed_parameter_bridge,
    ]

    # Node for launching camera robot state publisher
    robot_state_publisher_node_camera = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=camera_namespace,
        parameters=[camera_robot_description]
    )

    nodes_to_launch.append(robot_state_publisher_node_camera)

    # Node for spawning the camera in gazebo environment
    gazebo_spawn_camera_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=camera_namespace,
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-allow_renaming', 'false',
            '-x', '0.5',
            '-y', '0.9',
            '-z', '1.15',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-1.9',
            '-timeout', '10000',
        ],
        parameters=[{'use_sim_time': True}],
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[gazebo_spawn_camera_node]
    )

    nodes_to_launch.append(delayed_spawn)

    # Node for publishing environment pose
    env_pose_publisher = Node(
        package='xarm_gazebo',
        executable='env_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_pose_publisher = TimerAction(
        period=5.0,  # Delay 5 seconds to ensure Gazebo is fully loaded
        actions=[env_pose_publisher]
    )

    nodes_to_launch.append(delayed_pose_publisher)

    def _launch_all_robots(context):
        return sum(
            [
                get_per_robot_stack(robot_idx, bool(load_controller_config.perform(context)))
                for robot_idx in range(int(num_robots_config.perform(context)))
            ],
            []
        )

    launch_all_robots = OpaqueFunction(function=_launch_all_robots)
    return LaunchDescription([
        DeclareLaunchArgument('camera_namespace', default_value='camera_01', description='Camera namespace'),
        DeclareLaunchArgument('load_controller', default_value='true', description='Whether to load controllers'),
        DeclareLaunchArgument('num_robots', default_value='1', description='Number of robots to spawn'),
    ] + nodes_to_launch + [launch_all_robots])