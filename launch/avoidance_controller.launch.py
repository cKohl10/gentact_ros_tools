from logging import Logger
from math import log
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


import os
import yaml
import subprocess
import serial
import glob
import time

def load_config(config_file_name, context):
    package_share = FindPackageShare('gentact_ros_tools').perform(context)
    config_file = os.path.join(package_share, 'config', config_file_name)
    
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    return config

def build_controller_nodes(config):
    controller_nodes = []
    if config['controller']['active']:
        param_file = PathJoinSubstitution([
            FindPackageShare('hiro_collision_avoidance_ros2'),
            'config',
            'fr3.yaml',
        ])
        avoidance_type = config['controller']['avoidance_type']
        movement_type = config['controller']['movement_type']
        sim = config['controller']['use_sim']
        controller_nodes.append(Node(
            package='hiro_collision_avoidance_ros2',
            executable='Main',
            name='avoidance_controller',
            output='screen',
            parameters=[param_file, {'avoidance_type': avoidance_type, 'movement_type': movement_type}],
        ),
    )
    return controller_nodes

def build_robot_nodes(config):
    robot_nodes = []
    use_sim = bool(config['controller'].get('use_sim', False))
    sim_flag = 'true' if use_sim else 'false'
    launch_arguments = {}
    if config['sensors']['active']:
        for sensor_key, sensor_config in config['sensors'].items():
            if isinstance(sensor_config, dict) and sensor_config.get('xacro', '') != '':
                xacro_path = sensor_config.get('xacro', '')
                launch_arguments[f'{sensor_key}'] = xacro_path

    namespace = config['robot']['namespace']

    if use_sim:
        launch_arguments.update({
            'arm_id': str(config['robot']['arm_id']),
            'arm_prefix': str(config['robot']['arm_prefix']),
            'namespace': str(config['robot']['namespace']),
            'urdf_file': str(config['robot']['urdf_file']),
            'ee_id': str(config['robot']['ee_id']),
            'load_gripper': str(config['robot']['load_gripper']),
        })
        robot_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gentact_ros_tools'), 'launch', 'simulation.launch.py'
                ])
            ),
            launch_arguments=launch_arguments.items(),
        ))
    else:
        launch_arguments.update({
            'arm_id': str(config['robot']['arm_id']),
            'arm_prefix': str(config['robot']['arm_prefix']),
            'namespace': str(config['robot']['namespace']),
            'urdf_file': str(config['robot']['urdf_file']),
            'robot_ip': str(config['robot']['robot_ip']),
            'load_gripper': str(config['robot']['load_gripper']),
            # 'use_fake_hardware': str(config['robot']['use_fake_hardware']),
            'fake_sensor_commands': str(config['robot']['fake_sensor_commands']),
            'joint_state_rate': str(config['robot']['joint_state_rate']),
            'enable_gazebo': sim_flag,
        })
        robot_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gentact_ros_tools'), 'launch', 'franka.launch.py'
                    ])
                ),
                launch_arguments=launch_arguments.items(),
            ))
    
        # Only spawn the controller if it's active
        if config['controller']['relay_active']:
            controller_name = config['controller']['name']
            controller_params_file = PathJoinSubstitution([
                FindPackageShare('hiro_collision_avoidance_ros2'), 'config', "controllers.yaml",
            ])
            robot_nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[controller_name, '--controller-manager-timeout', '30',
                               '--param-file', controller_params_file],
                    output='screen',
                )
            )

        
    return robot_nodes

def build_sensor_nodes(config, config_file_name):
    sensor_nodes = []
    if config['sensors']['active']:
        sensor_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gentact_ros_tools'), 'launch', 'sensors.launch.py'
                    ])
                ),
                launch_arguments={'config': config_file_name}.items(),
            ))
    return sensor_nodes


def build_viz_nodes(config):
    viz_nodes = []
    if config['visualization']['rviz']:
        viz_nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            emulate_tty=True,
            sigterm_timeout='5',
            arguments=['-d', config['visualization']['rviz_config']]
        ))


    if config['visualization']['foxglove']:
        print("=====Launching foxglove bridge=====")
        viz_nodes.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='log',
        ))

    return viz_nodes


def launch_setup(context, *args, **kwargs):
    # Get the config file name from launch configuration
    config_file_name = LaunchConfiguration('config').perform(context)
    config = load_config(config_file_name, context)

    # Build robot description
    robot_nodes = build_robot_nodes(config)
    controller_nodes = build_controller_nodes(config)
    sensor_nodes = build_sensor_nodes(config, config_file_name)
    viz_nodes = build_viz_nodes(config)

    # Build launch actions list
    launch_actions = []
    launch_actions.extend(launch_nodes(viz_nodes, 0.0))
    launch_actions.extend(launch_nodes(robot_nodes, 2.0))
    launch_actions.extend(launch_nodes(sensor_nodes, 1.0))
    launch_actions.extend(launch_nodes(controller_nodes, 5.0))

    return launch_actions

def launch_nodes(nodes, timer_period):
    launch_actions = []
    if nodes is None:
        return launch_actions
    for node in nodes:
        launch_actions.append(TimerAction(period=timer_period, actions=[node]))
    return launch_actions

def generate_launch_description():

    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value='control_test.yaml',
        description='Configuration file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])

