#!/usr/bin/env python3

import os
import xacro
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

def build_robot_description(
    this_robot_prefix="",
    this_robot_namespace=""
):
    # which ros2 control plugin to use. could be ign control in the future if using ign gazebo
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='ign_ros2_control/IgnitionSystem')

    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config', 'xarm6_controllers.yaml'
        ),
        prefix=this_robot_prefix, 
        add_gripper=False, #TODO: fix this later
        add_bio_gripper=False,
        ros_namespace=this_robot_namespace,
        update_rate=1000,
        robot_type='xarm'
    )

    print(f"Generated temporary control params at {ros2_control_params}")

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    robot_description = {
        'robot_description': get_xacro_command(
            xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']), 
            mappings={
                'prefix': this_robot_prefix,
                'hw_ns': this_robot_namespace,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
            }
        ),
    }

    return robot_description

def build_camera_description(this_robot_namespace=""):
    robot_description = {
        'robot_description': get_xacro_command(
            xacro_file=PathJoinSubstitution([FindPackageShare('main'), 'urdf','camera', 'sensor_d455.urdf.xacro']), 
            mappings={
                'prefix': this_robot_namespace,
            }
        ),
    }
    return robot_description

def get_per_robot_stack(robot_idx, load_controller):
    """
    The following happens for each robot
    """
    this_robot_namespace = f""
    this_robot_prefix = f"xarm6_"
    nodes_to_launch = []

    robot_description = build_robot_description(
        this_robot_prefix=this_robot_prefix, 
        this_robot_namespace=this_robot_namespace
    )

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

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=this_robot_namespace,
        output='screen',
        arguments=[
            '-topic', f'robot_description',
            '-allow_renaming', 'false',
            '-x', str(0.0 + robot_idx * 0.4),
            '-y', '-0.3',
            '-z', '1.021',
            '-Y', '1.571',
            '-timeout', '10000',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # moveit_config_package_name = 'xarm_moveit_config'
    # xarm_type='xarm6' #TODO: fix
    
    # # robot_description_parameters
    # mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))

    # load_yaml = getattr(mod, 'load_yaml')
    # controllers_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, 'fake_controllers.yaml')
    # ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')

    # # Planning Configuration
    # ompl_planning_pipeline_config = {
    #     'default_planning_pipeline': 'ompl',
    #     'planning_pipelines': ['ompl'],
    # }
    # if os.environ.get('ROS_DISTRO', '') > 'iron':
    #     ompl_planning_pipeline_config['ompl'] = {
    #         'planning_plugins': ['ompl_interface/OMPLPlanner'],
    #         'request_adapters': [
    #             'default_planning_request_adapters/ResolveConstraintFrames',
    #             'default_planning_request_adapters/ValidateWorkspaceBounds',
    #             'default_planning_request_adapters/CheckStartStateBounds',
    #             'default_planning_request_adapters/CheckStartStateCollision',
    #         ],
    #         'response_adapters': [
    #             'default_planning_response_adapters/AddTimeOptimalParameterization',
    #             'default_planning_response_adapters/ValidateSolution',
    #             'default_planning_response_adapters/DisplayMotionPath',
    #         ],
    #     }
    # else:
    #     ompl_planning_pipeline_config['ompl'] = {
    #         'planning_plugin': 'ompl_interface/OMPLPlanner',
    #         'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         'start_state_max_bounds_error': 0.1,
    #     }
    # ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # # Moveit controllers Configuration
    # moveit_controllers = {
    #     'moveit_fake_controller_manager': controllers_yaml,
    #     'moveit_controller_manager': 'moveit_fake_controller_manager/MoveItFakeControllerManager',
    # }

    # # Trajectory Execution Configuration
    # trajectory_execution = {
    #     'moveit_manage_controllers': True,
    #     'trajectory_execution.allowed_execution_duration_scaling': 1.2,
    #     'trajectory_execution.allowed_goal_duration_margin': 0.5,
    #     'trajectory_execution.allowed_start_tolerance': 0.01,
    #     'trajectory_execution.execution_duration_monitoring': False
    # }

    # plan_execution = {
    #     'plan_execution.record_trajectory_state_frequency': 10.0,
    # }

    # planning_scene_monitor_parameters = {
    #     'publish_planning_scene': True,
    #     'publish_geometry_updates': True,
    #     'publish_state_updates': True,
    #     'publish_transforms_updates': True,
    #     # "planning_scene_monitor_options": {
    #     #     "name": "planning_scene_monitor",
    #     #     "robot_description": "robot_description",
    #     #     "joint_state_topic": "/joint_states",
    #     #     "attached_collision_object_topic": "/move_group/planning_scene_monitor",
    #     #     "publish_planning_scene_topic": "/move_group/publish_planning_scene",
    #     #     "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
    #     #     "wait_for_initial_state_timeout": 10.0,
    #     # },
    # }

    # Start the actual move_group node/action server
    # move_group_node = Node(
    #     package='moveit_ros_move_group',
    #     executable='move_group',
    #     namespace=this_robot_namespace, 
    #     output='screen',
    #     parameters=[
    #         '-topic', f'robot_description',
    #         ompl_planning_pipeline_config,
    #         trajectory_execution,
    #         plan_execution,
    #         moveit_controllers,
    #         planning_scene_monitor_parameters,
    #         {'use_sim_time': True},
    #     ],
    #     remappings=[
    #         ('/move_group/monitored_planning_scene', f'/{this_robot_namespace}/move_group/monitored_planning_scene'),
    #         ('/move_group/display_planned_path', f'/{this_robot_namespace}/move_group/display_planned_path'),
    #         # Add more remappings as needed
    #     ]
    # )

    # # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if False == 'true' else 'moveit.rviz'])
    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2_unique_name',
    #     output='screen',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[
    #         '-topic', f'robot_description',
    #         ompl_planning_pipeline_config,
    #         {'use_sim_time': True},
    #     ],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static'),
    #     ]
    # )

    nodes_to_launch.append(
        gazebo_spawn_entity_node,
    )

    # nodes_to_launch.append(
    #     rviz2_node,
    # )
    
    # nodes_to_launch.append(
    #     move_group_node,
    # )
    # # Load controllers
    controllers = [
        'joint_state_broadcaster',
        # the below becomes something like xarm0_xarm6_traj_controller. 
        # The first "xarm0" is from prefix. 
        # The second needs to match what's in xarm_control/config/*.yaml 
        f'{this_robot_prefix}traj_controller',
    ]
    # TODO: fix gripper loading for controllers

    # if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true'):
    #     controllers.append(
    #         f'{prefix.perform(context)}{robot_type.perform(context)}_gripper_traj_controller'
    #     )
    # elif robot_type.perform(context) != 'lite' and add_bio_gripper.perform(context) in ('True', 'true'):
    #     controllers.append(
    #         f'{prefix.perform(context)}bio_gripper_traj_controller'
    #     )

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
                    target_action=gazebo_spawn_entity_node,
                    on_exit=load_controllers
                )
            )
        )
    return nodes_to_launch

def generate_launch_description():

    num_robots_config = LaunchConfiguration("num_robots", default=1)
    num_robots_arg = DeclareLaunchArgument("num_robots", default_value="1", description="number of robots to launch")

    load_controller_config = LaunchConfiguration("load_controller", default=True)
    load_controller_arg = DeclareLaunchArgument("load_controller", default_value="True", description="whether to load joint controllers")

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    # xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
    #     launch_arguments={
    #         'gz_args': '-r'
    #     }
    # )

    world_sdf_path = os.path.join(get_package_share_directory('main'),'world','world.sdf') # Path to your world file
    camera_namespace = LaunchConfiguration('camera_namespace', default="camera_01")

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': f'-v4 -r {world_sdf_path}',  # Load the world file with Gazebo directly
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

    nodes_to_launch = [gazebo_launch, clock_parameter_bridge]

    # camera_robot_description = build_camera_description(this_robot_namespace = camera_namespace)

    # robot_state_publisher_node_camera = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     namespace=camera_namespace,
    #     parameters=[camera_robot_description]
    # )
    # nodes_to_launch.append(robot_state_publisher_node_camera)
    
    # gazebo_spawn_camera_node = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     namespace=camera_namespace,
    #     output='screen',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-allow_renaming', 'false',
    #         '-x', '0.2',  # Corrected: Added missing commas
    #         '-y', '0.7',
    #         '-z', '1.2',
    #         '-P', '0',
    #         '-Y', '-1.8',
    #         '-timeout', '10000'
    #     ],
    #     parameters=[{'use_sim_time': True}],
    # )

    # delayed_spawn = TimerAction(
    #     period=2.0,
    #     actions=[gazebo_spawn_camera_node]
    # )

    # nodes_to_launch.append(delayed_spawn)

    def _launch_all_robots(context):
        return sum(
            [
                get_per_robot_stack(
                    robot_idx, 
                    bool(load_controller_config.perform(context))
                ) for robot_idx in range(int(num_robots_config.perform(context)))
            ], [])
    
    launch_all_robots = OpaqueFunction(function=_launch_all_robots)
    return LaunchDescription(nodes_to_launch + [launch_all_robots] + [load_controller_arg, num_robots_arg])
