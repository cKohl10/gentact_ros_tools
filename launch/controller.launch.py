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
    package_share = FindPackageShare('gentact_ros_tools_hybrid').perform(context)
    config_file = os.path.join(package_share, 'config', config_file_name)
    
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    return config

def build_robot_description(config):
    """Build URDF arguments based on active sensors in config"""
    urdf_args = []
    
    # Loop through all sensors in the config
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('xacro', '') != '':
            xacro_path = sensor_config.get('xacro', '')
            urdf_args.extend([f' {sensor_key}:=', xacro_path])

    # Add end effector mesh if specified in config
    if isinstance(config['robot']['end_effector'], dict) and config['robot']['end_effector'].get('active', False):
        ee_xacro = config['robot']['end_effector'].get('xacro', '')
        if ee_xacro:
            urdf_args.extend([' ee_xacro_file:=', ee_xacro])
    
    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools_hybrid'), config['robot']['robot_xacro']])
    xacro_command = ['xacro ', urdf_file] + urdf_args
    robot_description = ParameterValue(
        Command(xacro_command), 
        value_type=str
    )

    return robot_description

def build_robot(config, use_sim_time, robot_description):
    robot_nodes = []

    robot_nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{config["robot"]["arm_id"]}_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    ))
    robot_nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base']
    ))
    if config['robot']['joint_publisher']:
        robot_nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='robot_joint_states',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ))
    return robot_nodes

def build_controller_nodes(config, robot_description):
    controller_nodes = []
    param_file = PathJoinSubstitution([
        FindPackageShare('gentact_ros_tools_hybrid'),
        'config',
        'fr3.yaml',
    ])
    if config['controller']['active']:
        avoidance_type = config['controller']['avoidance_type']
        movement_type = config['controller']['movement_type']
        sim = config['controller']['use_sim']
        controller_nodes.append(Node(
            package='hiro_collision_avoidance_ros2',
            executable='Main',
            name='avoidance_controller',
            output='screen',
            parameters=[param_file, {'avoidance_type': avoidance_type, 'movement_type': movement_type, 'robot_description': robot_description}],
        )
        )
    return controller_nodes

def build_sim_nodes(config):
    sim_nodes = []
    if config['controller']['use_sim']:
        sim_nodes.append(Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gzclient',
            output='screen',
        ))
        sim_nodes.append(Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_factory.so', 'worlds/empty.world'],
            output='screen'
        ))
    return sim_nodes

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

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build robot description
    robot_description = build_robot_description(config)
    robot_nodes = build_robot(config, use_sim_time, robot_description)
    controller_nodes = build_controller_nodes(config, robot_description)
    sim_nodes = build_sim_nodes(config)
    viz_nodes = build_viz_nodes(config)
    
    timer_period = 0.0
    timer_period_delay = 0.0

    # Build launch actions list
    launch_actions = []
    # launch_actions.extend(launch_nodes(robot_nodes, timer_period, timer_period_delay))
    # launch_actions.extend(launch_nodes(sim_nodes, timer_period, timer_period_delay))
    launch_actions.extend(launch_nodes(viz_nodes, timer_period, timer_period_delay))
    launch_actions.extend(launch_nodes(controller_nodes, timer_period, timer_period_delay))

    return launch_actions

def launch_nodes(nodes, timer_period, timer_period_delay):
    launch_actions = []
    for node in nodes:
        launch_actions.append(TimerAction(period=timer_period, actions=[node]))
        timer_period += timer_period_delay
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

