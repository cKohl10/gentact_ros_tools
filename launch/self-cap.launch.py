from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
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
            urdf_args.extend([f' {sensor_key}:=', xacro_path])

    # Add end effector mesh if specified in config
    if isinstance(config['robot']['end_effector'], dict) and config['robot']['end_effector'].get('active', False):
        ee_xacro = config['robot']['end_effector'].get('xacro', '')
        if ee_xacro:
            urdf_args.extend([' ee_xacro_file:=', ee_xacro])
    
    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), config['robot']['robot_xacro']])
    xacro_command = ['xacro ', urdf_file] + urdf_args
    robot_description = ParameterValue(
        Command(xacro_command), 
        value_type=str
    )

    return robot_description

def build_sensor_nodes(config):
    """Build sensor publisher nodes based on active sensors in config"""
    sensor_nodes = []
    
    # Loop through all sensors in the config
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('active', False):
            # Create a sensor publisher node for each active sensor
            sensor_node = Node(
                package='gentact_ros_tools',
                executable='sensor_publisher',
                name=f'{sensor_key}_publisher',
                parameters=[{
                    'port': sensor_config.get('port', '/dev/ttyACM0'),
                    'wireless': sensor_config.get('wireless', False),
                    'link_name': sensor_key.replace('_skin', ''),
                    'publish_rate': config['sensors'].get('publish_rate', 30.0),
                    'num_sensors': sensor_config.get('num_sensors', 0),
                }],
                output='screen'
            )
            sensor_nodes.append(sensor_node)
    
    return sensor_nodes

def build_prediction_nodes(config):
    # Prediction nodes based on config
    prediction_nodes = []
    
    # Add prediction nodes if active
    if 'prediction' in config and config['prediction'].get('active', False):
        prediction_config = config['prediction']
        
        # Add PCL prediction if active
        if prediction_config.get('pcl', {}).get('active', False):
            pcl_node = Node(
                package='gentact_ros_tools',
                executable='pcl_prediction',
                name='pcl_prediction',
                parameters=[{
                    'alpha': prediction_config['pcl'].get('alpha', 0.003),
                    'min_dist': prediction_config['pcl'].get('min_dist', 0.01),
                }],
                output='screen'
            )
            prediction_nodes.append(pcl_node)
        
        # Add MLP prediction if active
        if prediction_config.get('mlp', {}).get('active', False):
            mlp_node = Node(
                package='gentact_ros_tools',
                executable='mlp_prediction',
                name='mlp_prediction',
                parameters=[{
                    'model_path': prediction_config['mlp'].get('model_path', ''),
                }],
                output='screen'
            )
            prediction_nodes.append(mlp_node)
        
        # Add Mamba prediction if active
        if prediction_config.get('mamba', {}).get('active', False):
            mamba_node = Node(
                package='gentact_ros_tools',
                executable='mamba_prediction',
                name='mamba_prediction',
                parameters=[{
                    'model_path': prediction_config['mamba'].get('model_path', ''),
                }],
                output='screen'
            )
            prediction_nodes.append(mamba_node)
    
    return prediction_nodes

def build_calibration_nodes(config, use_sim_time):
    calibration_nodes = []
    if 'calibration' in config and config['calibration'].get('active', False):
        calibration_config = config['calibration']

        skin_description = ParameterValue(Command(['xacro ', PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), calibration_config['xacro']])]),value_type=str)
        skin_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='skin_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': skin_description}],
            remappings=[
                ('/joint_states', '/joint_states_skin'),
                ('/robot_description', '/robot_description_skin')
            ]
        )

        reference_point_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='reference_point_node',
            output='screen',
            arguments=['0.5', '0', '-0.04', '0', '0', '0', 'map', 'reference_point'] #Hardcoded for hiro env
        )

        calibration_base_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='calibration_base_node',
            output='screen',
            arguments=[str(x) for x in calibration_config['transform']]
        )

        if calibration_config.get('live_sensor', False):
            calibration_sensor_node = Node(
                package='gentact_ros_tools',
                executable='sensor_publisher',
                name=f'calibration_sensor_publisher',
                parameters=[{
                    'port': calibration_config.get('port', '/dev/ttyACM0'),
                    'wireless': calibration_config.get('wireless', False),
                    'link_name': calibration_config.get('link_name', 'calibration_sensor'),
                    'publish_rate': calibration_config.get('publish_rate', 30.0),
                    'num_sensors': calibration_config.get('num_sensors', 0),
                }],
                output='screen'
            )
            calibration_nodes.append(calibration_sensor_node)

        if calibration_config.get('process', False):
            calibration_process_node = Node(
                package='gentact_ros_tools',
                executable='processor',
                name='calibration_processor',
                parameters=[{
                    'num_sensors': calibration_config.get('num_sensors', 6),
                    'use_sim_time': use_sim_time,
                }],
                output='screen'
            )
            calibration_nodes.append(calibration_process_node)

            calibration_training_data_processor_node = Node(
                package='gentact_ros_tools',
                executable='training_data_processor',
                name='calibration_training_data_processor',
                parameters=[{
                    'num_sensors': calibration_config.get('num_sensors', 6),
                    'use_sim_time': use_sim_time,
                }],
                output='screen'
            )
            calibration_nodes.append(calibration_training_data_processor_node)
        
        calibration_nodes.append(reference_point_node)
        calibration_nodes.append(calibration_base_node)
        calibration_nodes.append(skin_state_publisher_node)

    return calibration_nodes

def build_joint_relay_nodes(config):
    joint_relay_nodes = []
    if config['robot']['joint_relay']:
        joint_relay_nodes.append(Node( # Controls the franka to mimic /joint_states_{arm_id}
            package='gentact_ros_tools',
            executable='franky_relay',
            name='franky_relay',
            output='screen',
            parameters=[{
                'robot_ip': config['robot']['robot_ip'],
                'arm_id': config['robot']['arm_id'],
            }]
        ))

        joint_relay_nodes.append(Node( # Relays /joint_states to the robot's namespace
            package='gentact_ros_tools',
            executable='panda2fr3',
            name='panda2fr3',
            output='screen',
            parameters=[{
                'arm_id': config['robot']['arm_id'],
            }]
        ))

    return joint_relay_nodes

def build_viz_nodes(config):
    viz_nodes = []
    if config['visualization']['rviz']:
        viz_nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'config': config['visualization']['rviz_config'],
            }]
        ))

    if config['visualization']['foxglove']:
        viz_nodes.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ))

    return viz_nodes

def build_cameras_nodes(config):
    cameras_nodes = []
    if config['cameras']['active']:
        cameras_launch_file = config['cameras']['launch_file']
        cameras_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gentact_ros_tools'),
                    'launch',
                    cameras_launch_file
                ])
            ])
        )
        cameras_nodes.append(cameras_launch)
    return cameras_nodes

def launch_setup(context, *args, **kwargs):
    # Get the config file name from launch configuration
    config_file_name = LaunchConfiguration('config').perform(context)
    config = load_config(config_file_name, context)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build robot description
    robot_description = build_robot_description(config)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{config["robot"]["arm_id"]}_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )
    robot_st_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base']
    )
    
    # Build sensor nodes dynamically from config
    sensor_nodes = build_sensor_nodes(config)
    prediction_nodes = build_prediction_nodes(config)
    viz_nodes = build_viz_nodes(config)
    camera_nodes = build_cameras_nodes(config)
    calibration_nodes = build_calibration_nodes(config, use_sim_time)
    joint_relay_nodes = build_joint_relay_nodes(config)

    # Build launch actions list
    launch_actions = [
        robot_state_publisher_node,
        robot_st_base_node,
    ]

    # Add visualization nodes
    for viz_node in viz_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[viz_node]))

    # Add camera nodes with delays
    for camera_node in camera_nodes:
        launch_actions.append(TimerAction(period=2.0, actions=[camera_node]))

    # Add calibration nodes with delays
    for calibration_node in calibration_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[calibration_node]))
    
    # Add sensor nodes with delays
    for sensor_node in sensor_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[sensor_node]))
    
    # Add prediction nodes with delays
    for prediction_node in prediction_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[prediction_node]))

    # Add joint relay nodes with delays
    for joint_relay_node in joint_relay_nodes:
        launch_actions.append(TimerAction(period=5.0, actions=[joint_relay_node]))

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
