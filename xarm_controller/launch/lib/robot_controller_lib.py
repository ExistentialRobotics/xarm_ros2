#!/usr/bin/env python3

import os
import yaml
from tempfile import NamedTemporaryFile
from ament_index_python import get_package_share_directory


def add_prefix_to_ros2_control_params(prefix, ros2_control_params):
    if not prefix:
        return
    for name in list(ros2_control_params.keys()):
        if name == 'controller_manager':
            continue
        ros__parameters = ros2_control_params[name].get('ros__parameters', {})
        joints = ros__parameters.get('joints', [])
        constraints = ros__parameters.get('constraints', {})
        for i, joint in enumerate(joints):
            for j, key in enumerate(constraints.keys()):
                if key == joint:
                    constraints['{}{}'.format(prefix, key)] = constraints.pop(key)
                    break
            joints[i] = '{}{}'.format(prefix, joint)
        new_name = '{}{}'.format(prefix, name)
        ros2_control_params[new_name] = ros2_control_params.pop(name)
        controller_manager_ros__parameters = ros2_control_params.get('controller_manager', {}).get('ros__parameters', {})
        if name in controller_manager_ros__parameters:
            controller_manager_ros__parameters[new_name] = controller_manager_ros__parameters.pop(name)


def generate_ros2_control_params_temp_file(ros2_control_params_path, prefix='', add_gripper=False, add_bio_gripper=False, ros_namespace='', update_rate=None, robot_type='xarm'):
    if ros_namespace or prefix or update_rate:
        with open(ros2_control_params_path, 'r') as f:
            ros2_control_params_yaml = yaml.safe_load(f)
        if update_rate is not None:
            ros2_control_params_yaml['controller_manager']['ros__parameters']['update_rate'] = update_rate
        if add_gripper:
            gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_gripper_controllers.yaml'.format(robot_type))
            # check file is exists or not
            if os.path.exists(gripper_control_params_path):
                with open(gripper_control_params_path, 'r') as f:
                    gripper_control_params_yaml = yaml.safe_load(f)
                for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                    ros2_control_params_yaml['controller_manager']['ros__parameters'][name] = value
                    if name in gripper_control_params_yaml:
                        ros2_control_params_yaml[name] = gripper_control_params_yaml[name]
        elif add_bio_gripper:
            gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', 'bio_gripper_controllers.yaml')
            # check file is exists or not
            if os.path.exists(gripper_control_params_path):
                with open(gripper_control_params_path, 'r') as f:
                    gripper_control_params_yaml = yaml.safe_load(f)
                for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                    ros2_control_params_yaml['controller_manager']['ros__parameters'][name] = value
                    if name in gripper_control_params_yaml:
                        ros2_control_params_yaml[name] = gripper_control_params_yaml[name]
                
        add_prefix_to_ros2_control_params(prefix, ros2_control_params_yaml)

        if not ros_namespace.startswith('/'):
            ros_namespace = f'/{ros_namespace}'

        # add namespace, possibly just slash to all nodes
        namespaced_ros2_control_params = {
            f'{ros_namespace}/{key}': val
            for key, val in ros2_control_params_yaml.items()
        }

        with NamedTemporaryFile(mode='w', prefix='controller_params_', delete=False) as h:
            # specify the params_file for all the controllers, as the temporary saved file.
            controller_spec = namespaced_ros2_control_params[f"{ros_namespace}/controller_manager"]["ros__parameters"]
            for potential_controller in controller_spec.keys():
                if potential_controller == "update_rate":
                    continue
                controller_spec[potential_controller]["params_file"] = h.name
            
            yaml.dump(namespaced_ros2_control_params, h, default_flow_style=False)
            return h.name
    return ros2_control_params_path


def generate_dual_ros2_control_params_temp_file(
    ros2_control_params_path_1, ros2_control_params_path_2, 
    prefix_1='L_', prefix_2='R_', 
    add_gripper_1=False, add_gripper_2=False, 
    add_bio_gripper_1=False, add_bio_gripper_2=False, 
    ros_namespace='', update_rate=None,
    robot_type_1='xarm', robot_type_2='xarm'):
    with open(ros2_control_params_path_1, 'r') as f:
        ros2_control_params_yaml_1 = yaml.safe_load(f)
    with open(ros2_control_params_path_2, 'r') as f:
        ros2_control_params_yaml_2 = yaml.safe_load(f)
    if update_rate is not None:
        ros2_control_params_yaml_1['controller_manager']['ros__parameters']['update_rate'] = update_rate
        ros2_control_params_yaml_2['controller_manager']['ros__parameters']['update_rate'] = update_rate

    if add_gripper_1:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_gripper_controllers.yaml'.format(robot_type_1))
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_1['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_1[name] = gripper_control_params_yaml[name]
    elif add_bio_gripper_1:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', 'bio_gripper_controllers.yaml')
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_1['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_1[name] = gripper_control_params_yaml[name]
        
    if add_gripper_2:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_gripper_controllers.yaml'.format(robot_type_2))
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_2['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_2[name] = gripper_control_params_yaml[name]
    elif add_bio_gripper_2:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', 'bio_gripper_controllers.yaml')
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_2['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_2[name] = gripper_control_params_yaml[name]
        
    add_prefix_to_ros2_control_params(prefix_1, ros2_control_params_yaml_1)
    add_prefix_to_ros2_control_params(prefix_2, ros2_control_params_yaml_2)
    ros2_control_params_yaml = {}
    ros2_control_params_yaml.update(ros2_control_params_yaml_1)
    ros2_control_params_yaml.update(ros2_control_params_yaml_2)
    ros2_control_params_yaml['controller_manager']['ros__parameters'].update(ros2_control_params_yaml_1['controller_manager']['ros__parameters'])
    ros2_control_params_yaml['controller_manager']['ros__parameters'].update(ros2_control_params_yaml_2['controller_manager']['ros__parameters'])
    if ros_namespace:
        ros2_control_params_yaml = {
            ros_namespace: ros2_control_params_yaml
        }
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
        yaml.dump(ros2_control_params_yaml, h, default_flow_style=False)
        return h.name