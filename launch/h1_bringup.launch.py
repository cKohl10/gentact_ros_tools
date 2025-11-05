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

def build_robot_description():    
    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_descriptions'), 'robots/h1-2/h1_2_handless.urdf.xacro'])
    xacro_command = ['xacro ', urdf_file]
    robot_description = ParameterValue(
        Command(xacro_command), 
        value_type=str
    )

    return robot_description

def build_robot(use_sim_time, robot_description):
    robot_nodes = []

    robot_nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='h1_2_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    ))
    robot_nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'pelvis']
    ))
    robot_nodes.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='robot_joint_states',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    ))
    return robot_nodes

def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build robot description
    robot_description = build_robot_description()
    robot_nodes = build_robot(use_sim_time, robot_description)
    return robot_nodes

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

