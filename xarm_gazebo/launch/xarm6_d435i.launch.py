#!/usr/bin/env python3
"""
Spawn an xArm6 with a wrist-mounted Intel RealSense D435i in Ignition Gazebo,
and bridge the camera/depth/pointcloud topics to ROS 2.

Topics published after the bridge starts (under `<camera_ns>`):
  color/image            sensor_msgs/Image
  color/camera_info      sensor_msgs/CameraInfo
  depth/image            sensor_msgs/Image  (float32 metric depth)
  depth/camera_info      sensor_msgs/CameraInfo
  depth/points           sensor_msgs/PointCloud2
"""

import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    load_python_launch_file_as_module,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.uf_robot_utils import get_xacro_command


ROBOT_PREFIX = 'xarm6_'
ROBOT_NAMESPACE = ''


def build_robot_description():
    ros2_control_plugin = 'ign_ros2_control/IgnitionSystem'

    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'launch', 'lib', 'robot_controller_lib.py',
        )
    )
    generate_ros2_control_params_temp_file = getattr(
        mod, 'generate_ros2_control_params_temp_file'
    )
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config', 'xarm6_controllers.yaml',
        ),
        prefix=ROBOT_PREFIX,
        add_gripper=False,
        add_bio_gripper=False,
        ros_namespace=ROBOT_NAMESPACE,
        update_rate=1000,
        robot_type='xarm',
    )

    with open('ros2_control_params.yaml', 'w') as f:
        yaml.dump(ros2_control_params, f, sort_keys=False)

    return {
        'robot_description': get_xacro_command(
            xacro_file=PathJoinSubstitution([
                FindPackageShare('xarm_description'),
                'urdf', 'xarm_device.urdf.xacro',
            ]),
            mappings={
                'prefix': ROBOT_PREFIX,
                'hw_ns': ROBOT_NAMESPACE,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'robot_spec_config_file': PathJoinSubstitution([
                    FindPackageShare('xarm_description'),
                    'config', 'default_urdf_arguments', 'xarm6.yaml',
                ]),
                'end_effector_config_file': PathJoinSubstitution([
                    FindPackageShare('xarm_description'),
                    'config', 'default_urdf_arguments', 'end_effector_d435i.yaml',
                ]),
            },
        ),
    }


def _build_camera_bridge():
    """Bridge gz-sim sensor topics (set via <topic> in realsense.gz.xacro)
    to ROS 2.
    """
    stem = f'/{ROBOT_PREFIX}camera'
    args = [
        # RGB
        f'{stem}/color@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'{stem}/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        # Depth
        f'{stem}/depth@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'{stem}/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        f'{stem}/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ]
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrist_camera_bridge',
        output='screen',
        arguments=args,
        parameters=[{'use_sim_time': True}],
    )


def _build_camera_frame_aliases():
    """gz-sensors in Fortress ignores <gz_frame_id> for the depth_camera
    point cloud (gazebosim/gz-sensors#454). Messages get tagged with the
    auto-generated sensor scope `<model>/<link>/<sensor>` instead of the
    URDF frame. Publish static TFs aliasing those scope names onto the
    URDF link frames so rviz can resolve them.

    The model name is what `keti_gz_utils/create_on_table` registers
    (`xarm_device`), the link is the EEF chain link (`xarm6_link6`), and
    the sensor names come from realsense.gz.xacro
    (`xarm6_cameradepth`, `xarm6_cameracolor`, optional IR).
    """
    model = 'xarm_device'
    link = f'{ROBOT_PREFIX}link6'

    def static_tf(name, parent, child):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', parent, child],
            parameters=[{'use_sim_time': True}],
        )

    return [
        static_tf(
            'tf_alias_cameradepth',
            f'{ROBOT_PREFIX}camera_depth_frame',
            f'{model}/{link}/{ROBOT_PREFIX}cameradepth',
        ),
        static_tf(
            'tf_alias_cameracolor',
            f'{ROBOT_PREFIX}camera_color_frame',
            f'{model}/{link}/{ROBOT_PREFIX}cameracolor',
        ),
    ]


def _spawn_robot():
    # Use keti_gz_utils/create_on_table: it patches the SDF to add a fixed
    # `world -> <canonical_link>` joint before spawning, which anchors the
    # robot to the world frame. Plain `ros_gz_sim create` skips that step,
    # and the model free-falls.
    return Node(
        package='keti_gz_utils',
        executable='create_on_table',
        name='spawn_xarm6_d435i',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'topic': 'robot_description',
            'allow_renaming': False,
            'x': 0.0,
            'y': -0.5,
            'z': 1.021,
            'Y': -1.571,
        }],
    )


def _launch_setup(context, *args, **kwargs):
    robot_description = build_robot_description()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    rviz_config = os.path.join(
        '/keti_ws/src/xarm_ros2/xarm_gazebo/rviz', 'xarm6_d435i.rviz',
    )
    # rviz2 occasionally latches wall-clock at startup if /clock isn't being
    # published yet, then ignores all later sim-time messages. Pass the
    # parameter via --ros-args (more reliable than the parameters= dict) and
    # delay launch until the bridge is up — see delayed_rviz below.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '-p', 'use_sim_time:=true'],
        parameters=[{'use_sim_time': True}],
    )

    spawn = _spawn_robot()

    controllers = [
        'joint_state_broadcaster',
        f'{ROBOT_PREFIX}traj_controller',
    ]
    spawn_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                ctl,
                '--controller-manager', f'{ROBOT_NAMESPACE}/controller_manager',
                '--namespace', f'{ROBOT_NAMESPACE}',
            ],
            parameters=[{'use_sim_time': True}],
        )
        for ctl in controllers
    ]

    # Hold rviz back until after the bridge publishes /clock; otherwise it
    # can lock in wall-time at startup and drop every sim-time message.
    delayed_rviz = TimerAction(period=5.0, actions=[rviz])

    return [
        robot_state_publisher,
        delayed_rviz,
        *_build_camera_frame_aliases(),
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=spawn_controllers,
            )
        ),
    ]


def generate_launch_description():
    # gz Fortress depth_camera sensors require ogre2; the default
    # table_world.sdf uses ogre1 and returns zero depth.
    default_world = os.path.join(
        get_package_share_directory('xarm_gazebo'),
        'worlds', 'table.world',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': f' -r {default_world}'}.items(),
    )

    # Bridge needs the gz sensors to have advertised topics; give the spawn
    # a head start.
    delayed_bridge = TimerAction(
        period=3.0,
        actions=[_build_camera_bridge()],
    )

    return LaunchDescription([
        gazebo,
        OpaqueFunction(function=_launch_setup),
        delayed_bridge,
    ])
