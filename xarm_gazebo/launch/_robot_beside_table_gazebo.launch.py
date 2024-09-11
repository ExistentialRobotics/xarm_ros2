#!/usr/bin/env python3

import os
import xacro
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction

def get_robot_name(
    robot_type,
    dof
):
    return f"{robot_type}{dof if robot_type in ('xarm', 'lite') else ''}"

def build_robot_description(
    context,
    this_robot_prefix="",
    this_robot_namespace=""
):
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    # hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)

    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')

    # whether to add gripper
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)

    # whether to apply joint limits
    limited = LaunchConfiguration('limited', default=False)
    # whether to enable effort/velocity control
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    # which ros2 control plugin to use. could be ign control in the future if using ign gazebo
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='ign_ros2_control/IgnitionSystem')

    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    # 3rd party end-effector support
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config', f'{get_robot_name(robot_type.perform(context), dof.perform(context))}_controllers.yaml'
        ),
        prefix=this_robot_prefix, 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=this_robot_namespace,
        update_rate=1000,
        robot_type=robot_type.perform(context)
    )

    print(ros2_control_params)

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']), 
            arguments={
                'prefix': this_robot_prefix,
                'dof': dof,
                'robot_type': robot_type,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'add_bio_gripper': add_bio_gripper,
                'hw_ns': this_robot_namespace,
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'add_realsense_d435i': add_realsense_d435i,
                'add_d435i_links': add_d435i_links,
                'model1300': model1300,
                'robot_sn': robot_sn,
                'attach_to': attach_to,
                'attach_xyz': attach_xyz,
                'attach_rpy': attach_rpy,
                'add_other_geometry': add_other_geometry,
                'geometry_type': geometry_type,
                'geometry_mass': geometry_mass,
                'geometry_height': geometry_height,
                'geometry_radius': geometry_radius,
                'geometry_length': geometry_length,
                'geometry_width': geometry_width,
                'geometry_mesh_filename': geometry_mesh_filename,
                'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
                'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
                'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
                'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
                'kinematics_suffix': kinematics_suffix,
            }
        ),
    }

    return robot_description

def build_camera_description(context, this_robot_prefix="", this_robot_namespace=""):
    # Define the path to the camera xacro file
    camera_xacro_path = PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'camera', 'camera.gazebo.xacro'
    ]).perform(context)

    # Use xacro to process the xacro file
    doc = xacro.process_file(camera_xacro_path)
    camera_description_xml = doc.toprettyxml(indent='  ')

    # Return the camera description in the required format
    camera_description = {
        'camera_description': camera_description_xml
    }

    return camera_description

def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')        
    load_controller = LaunchConfiguration('load_controller', default=True)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    num_robots = LaunchConfiguration('num_robots', default=1)

    # ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    # xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
    # gazebo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
    #     launch_arguments={
    #         'gz_args': ''
    #     }.items(),
    # )

    # Path to your world file
    world_sdf_path = os.path.join(get_package_share_directory('main'),'world','world.sdf')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': f'-r {world_sdf_path}',  # Load the world file with Gazebo directly
        }.items(),
    )

    # ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
    clock_parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    # # Camera launch
    # camera_description = build_camera_description(context, this_robot_prefix="camera_01", this_robot_namespace=this_robot_namespace)

    # camera_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}, camera_description],
    #     namespace=this_robot_namespace
    # )

    # gazebo_spawn_camera_node = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     namespace=this_robot_namespace,
    #     output='screen',
    #     arguments=[
    #         '-topic', 'camera_description',
    #         '-allow_renaming', 'false',
    #         '-x', '0.4',
    #         '-y', '-0.54',
    #         '-z', '1.021',
    #         '-R', '-1.571', '-P', '0', '-Y', '1.571'
    #     ],
    #     parameters=[{'use_sim_time': True}],
    # )

    nodes_to_launch = [gazebo_launch, clock_parameter_bridge]
    # nodes_to_launch.append(camera_state_publisher_node)
    # nodes_to_launch.append(gazebo_spawn_camera_node)  
    last_spawn_entity = None 

    for robot_idx in range(int(num_robots.perform(context))):
        this_robot_namespace = f"xarm{robot_idx}"
        this_robot_prefix = f"{this_robot_namespace}_"
        # this_robot_namespace=""
        # this_robot_prefix=""


        robot_description = build_robot_description(
            context, 
            this_robot_prefix=this_robot_prefix, 
            this_robot_namespace=this_robot_namespace
        )

        """
        The following happens for each robot
        """
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

        nodes_to_launch.append(robot_state_publisher_node)

        # gazebo spawn entity node
        gazebo_spawn_entity_node = Node(
            package="ros_gz_sim",
            executable="create",
            namespace=this_robot_namespace,
            output='screen',
            arguments=[
                '-topic', f'robot_description',
                # '-entity', '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else ''),
                # '-entity', f'{this_robot_namespace}_robot',
                '-allow_renaming', 'false',
                '-x', str(-0.4 + robot_idx * 0.4),
                '-y', '-0.54' if robot_type.perform(context) == 'uf850' else '-0.5',
                '-z', '1.021',
                '-Y', '1.571',
                '-timeout', '10000',
                # this puts gazebo_ros2_control to correct namespace
                # '-robot_namespace', this_robot_namespace
            ],
            parameters=[{'use_sim_time': True}],
        )

        if last_spawn_entity is not None:
            nodes_to_launch.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_spawn_entity,
                        on_exit=gazebo_spawn_entity_node
                    )
                )
            )
        else:
            nodes_to_launch.append(
                gazebo_spawn_entity_node            
            )
        
        last_spawn_entity = gazebo_spawn_entity_node

        # Load controllers
        controllers = [
            # f'{this_robot_prefix}{get_robot_name(robot_type.perform(context), dof.perform(context))}_joint_state_broadcaster',
            'joint_state_broadcaster',
            # the below becomes something like xarm0_xarm6_traj_controller. 
            # The first "xarm0" is from prefix. 
            # The second needs to match what's in xarm_control/config/*.yaml 
            f'{this_robot_prefix}traj_controller',
        ]
        if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true'):
            controllers.append(
                f'{prefix.perform(context)}{robot_type.perform(context)}_gripper_traj_controller'
            )
        elif robot_type.perform(context) != 'lite' and add_bio_gripper.perform(context) in ('True', 'true'):
            controllers.append(
                f'{prefix.perform(context)}bio_gripper_traj_controller'
            )

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
        ] if load_controller.perform(context) in ('True', 'true') else []

        if load_controllers:
            nodes_to_launch.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=gazebo_spawn_entity_node,
                        on_exit=load_controllers
                    )
                )
            )

    return nodes_to_launch

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
