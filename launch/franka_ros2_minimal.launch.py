#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.198',
        description='IP address of the Franka robot'
    )
    
    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value='fr3',
        description='Arm ID for the robot'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configuration
    robot_ip = LaunchConfiguration('robot_ip')
    arm_id = LaunchConfiguration('arm_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Franka robot description
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('franka_description'),
        'robots',
        'panda_arm.urdf.xacro'
    ])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{arm_id}_robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_file
        }],
        remappings=[
            ('/joint_states', f'/joint_states_{arm_id}'),
            ('/robot_description', f'/robot_description_{arm_id}')
        ]
    )
    
    # Franka driver node
    franka_driver_node = Node(
        package='franka_ros2',
        executable='franka_control_node',
        name='franka_control',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
            'arm_id': arm_id,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint trajectory controller
    joint_trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller_spawner',
        output='screen',
        arguments=[f'{arm_id}_joint_trajectory_controller', '--controller-manager', f'{arm_id}_controller_manager']
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[f'{arm_id}_joint_state_broadcaster', '--controller-manager', f'{arm_id}_controller_manager']
    )
    
    # Custom controller node (your franka_ros2_controller.py)
    custom_controller_node = Node(
        package='gentact_ros_tools',
        executable='franka_ros2_controller',
        name='franka_ros2_controller',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
            'arm_id': arm_id,
            'velocity_scaling': 1.0,
            'update_rate_hz': 20.0
        }]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        arm_id_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        franka_driver_node,
        joint_trajectory_controller_node,
        joint_state_broadcaster_node,
        custom_controller_node
    ])
