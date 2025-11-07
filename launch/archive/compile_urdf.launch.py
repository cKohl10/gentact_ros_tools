from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml

def load_config(config_file_name, context):
    package_share = FindPackageShare('gentact_ros_tools').perform(context)
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
            urdf_args.append(f'{sensor_key}:={xacro_path}')

    # Add end effector mesh if specified in config
    if isinstance(config['robot']['end_effector'], dict) and config['robot']['end_effector'].get('active', False):
        ee_xacro = config['robot']['end_effector'].get('xacro', '')
        if ee_xacro:
            urdf_args.append(f'ee_xacro_file:={ee_xacro}')
    
    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), config['robot']['robot_xacro']])
    xacro_command = ['xacro ', urdf_file] + urdf_args
    robot_description = ParameterValue(
        Command(xacro_command), 
        value_type=str
    )

    return robot_description, urdf_file, urdf_args

def launch_setup(context, *args, **kwargs):
    # Get the config file name from launch configuration
    config_file_name = LaunchConfiguration('config').perform(context)
    config = load_config(config_file_name, context)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build robot description
    robot_description, urdf_file, urdf_args = build_robot_description(config)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{config["robot"]["arm_id"]}_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )
    
    # Resolve paths in context
    package_share = FindPackageShare('gentact_ros_tools').perform(context)
    urdf_file_resolved = urdf_file.perform(context)
    
    # Use source directory instead of install directory
    # Get the source directory by going up from package_share: install/gentact_ros_tools/share/gentact_ros_tools -> src/gentact_ros_tools
    install_dir = os.path.dirname(os.path.dirname(os.path.dirname(package_share)))  # Remove share/gentact_ros_tools
    workspace_root = os.path.dirname(install_dir)  # Go up one more level to get workspace root
    src_dir = os.path.join(workspace_root, 'src')
    source_package_dir = os.path.join(src_dir, 'gentact_ros_tools')
    
    output_file_resolved = os.path.join(source_package_dir, 'urdf', 'compiled', 'robot.urdf')
    output_dir_resolved = os.path.join(source_package_dir, 'urdf', 'compiled')
    
    # Ensure output directory exists
    mkdir_cmd = ['mkdir', '-p', output_dir_resolved]
    
    # Build the xacro command
    xacro_cmd = ['xacro', urdf_file_resolved, '-o', output_file_resolved] + urdf_args
    
    # Print the command for debugging
    print(f"Xacro command: {' '.join([str(arg) for arg in xacro_cmd])}")
    
    mkdir_process = ExecuteProcess(
        cmd=mkdir_cmd,
        output='screen',
        name='mkdir_compile_dir'
    )
    
    xacro_process = ExecuteProcess(
        cmd=xacro_cmd,
        output='screen',
        name='xacro_compile'
    )
    

    # Build launch actions list
    launch_actions = [
        mkdir_process,
        xacro_process,
    ]


    return launch_actions

def generate_launch_description():

    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value='simulation.yaml',
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
