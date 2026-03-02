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
        avoidance_type = config['controller']['avoidance_type']
        movement_type = config['controller']['movement_type']
        controller_nodes.append(Node(
            package='hiro_collision_avoidance_ros2',
            executable='Main',
            name='Main',
            output='screen',
            parameters=[{'avoidance_type': avoidance_type, 'movement_type': movement_type}],
        ))
    return controller_nodes

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

def build_sensor_nodes(config, sensor_port_mapping):
    """Build sensor publisher nodes based on active sensors in config"""
    sensor_nodes = []
    
    # Loop through all sensors in the config
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('active', False):

            if sensor_config.get('type', '') == "SPAD":
                # Create a sensor publisher node for each active sensor
                sensor_node = Node(
                    package='gentact_ros_tools',
                    executable='tof_pub_pc',
                    name=f'{sensor_key}_publisher',
                    output='screen',
                    parameters=[{
                        'num_sensors': sensor_config.get('num_sensors', 8),
                        'publish_rate': config['sensors'].get('publish_rate', 30.0),
                    }]
                )
                print(f"SPAD Sensor publisher for {sensor_key} built")
            elif sensor_config.get('type', '') == "SCPS":
                link_name = sensor_key.replace('_skin', '')
                
                # Use scanned port if available, otherwise fallback to config or default
                serial_port = sensor_port_mapping.get(sensor_key, sensor_config.get('port', ''))

                if serial_port == '':
                    print(f"Sensor publisher for {sensor_key} skipped")
                    continue
                
                sensor_node = Node(
                    package='gentact_ros_tools',
                    executable='sensor_publisher',
                    name=f'{sensor_key}_publisher',
                    parameters=[{
                        'serial_port': serial_port,
                        'wireless': sensor_config.get('wireless', False),
                        'link_name': link_name,
                        'publish_rate': config['sensors'].get('publish_rate', 30.0),
                        'num_sensors': sensor_config.get('num_sensors', 0),
                        'skin_name': link_name,
                    }],
                    output='screen'
                )

                print(f"SCPS Sensor publisher for {sensor_key} built")
            sensor_nodes.append(sensor_node)
    return sensor_nodes

def build_udp_listener_nodes(config):
    udp_listener_nodes = []
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('type', '') == "SPAD" and sensor_config.get('active', False):
            udp_listener_nodes.append(Node(
                package='udp_tof_listener',
                executable='udp_grid_listener_array',
                name='udp_listener',
                output='screen'
            ))
    return udp_listener_nodes


def build_prediction_nodes(config, sensor_key, sensor_config):
    # Prediction nodes based on config for a specific sensor
    prediction_nodes = []
    # print(f"Building prediction nodes for {sensor_key}")
    
    # Add prediction nodes if active
    if 'prediction' in config and config['prediction'].get('active', False):
        prediction_config = config['prediction']
        
        # Add PCL prediction if active
        if prediction_config.get('pcl', {}).get('active', False):
            # Extract link name from sensor key (remove _skin suffix if present)
            link_name = sensor_key.replace('_skin', '')

            if sensor_config.get('alpha', None) is not None:
                alpha = sensor_config.get('alpha', None)
            else:
                alpha = prediction_config['pcl'].get('alpha', 0.003)

            if sensor_config.get('max_distance', None) is not None:
                max_distance = sensor_config.get('max_distance', None)
            else:
                max_distance = prediction_config['pcl'].get('max_distance', 0.1)
            
            # Build parameters dict, only including multiplier if it's defined
            params = {
                'frame_id': link_name,
                'num_sensors': sensor_config.get('num_sensors', 0),
                'skin_name': link_name,
                'alpha': alpha,
                'max_distance': max_distance,
            }
            
            # Only add multiplier parameter if it's actually defined in config
            if 'multiplier' in sensor_config:
                params['multiplier'] = sensor_config['multiplier']
            
            pcl_node = Node(
                package='gentact_ros_tools',
                executable='capacitive_pcl',
                name=f'pcl_prediction_{link_name}',
                parameters=[params],
                output='screen'
            )
            prediction_nodes.append(pcl_node)
        
    

        aggregated_obstacles_node = Node(
            package='gentact_ros_tools',
            executable='closest_obstacle',
            name=f'closest_obstacle_pub',
            output='screen'
        )
        prediction_nodes.append(aggregated_obstacles_node)
    
    return prediction_nodes


def build_tracker_nodes(config, sensor_key, sensor_config):
    tracker_nodes = []
    if 'tracker' in config and config['tracker'].get('active', False):
            # Build parameters dict, only including multiplier if it's defined
            params = {
                'num_sensors': sensor_config.get('num_sensors', 0),
                'skin_name': sensor_key.replace('_skin', ''),
                'Kp': config['tracker'].get('Kp', 0.15),
                'Kd': config['tracker'].get('Kd', 0.001),
                'baseline_kp': config['tracker'].get('baseline_kp', 0.01),
                'baseline_ki': config['tracker'].get('baseline_ki', 0.0000001),
                'baseline_kd': config['tracker'].get('baseline_kd', 0.000001),
                'baseline_timeout': config['tracker'].get('baseline_timeout', 10.0),
                'baseline_duration': config['tracker'].get('baseline_duration', 3.0),
                'masking_threshold': config['tracker'].get('masking_threshold', 200.0),
            }
            tracker_node = Node(
                package='gentact_ros_tools',
                executable='sensor_tracking_pub',
                name=f'sensor_tracking_pub_{sensor_key}',
                output='screen',
                parameters=[params]
            )
            tracker_nodes.append(tracker_node)
    return tracker_nodes

def build_joint_relay_nodes(config):
    joint_relay_nodes = []
    if config['robot']['joint_relay']:
        # joint_relay_nodes.append(Node( # Controls the franka to mimic /joint_states_{arm_id}
        #     package='gentact_ros_tools',
        #     executable='franky_relay',
        #     name='franky_relay',
        #     output='screen',
        #     parameters=[{
        #         'robot_ip': config['robot']['robot_ip'],
        #         'arm_id': config['robot']['arm_id'],
        #     }]
        # ))

        joint_relay_nodes.append(Node( # Relays /joint_states to the robot's namespace
            package='gentact_ros_tools',
            executable='panda2fr3',
            name='panda2fr3',
            output='screen',
            parameters=[{
                'arm_id': config['robot']['arm_id'],
            }]
        ))

        if config['robot']['joint_publisher']:
            joint_relay_nodes.append(Node( # Relays /joint_states to the robot's namespace
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui'
            ))

    return joint_relay_nodes

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

"""
def build_tuner_nodes(config):
    tuner_nodes = []
    if config['avoidance']['active']:
        tuner_nodes.append(Node(
            package='gentact_ros_tools',
            executable='tuner',
            name='rviz2',
            output='screen',
            parameters=[{
                'config': config['visualization']['rviz_config'],
            }]
        ))
"""
        
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
    robot_nodes = build_robot(config, use_sim_time, robot_description)
    controller_nodes = build_controller_nodes(config)
    
    # Build sensor nodes dynamically from config
    # Get port mapping from serial number scanning
    sensor_port_mapping = map_serial_numbers_to_sensors(config)
    print(f"Completed mapping serial numbers to sensor ports\n\n")
    sensor_nodes = build_sensor_nodes(config, sensor_port_mapping)
    
    # Build prediction nodes for each active sensor
    prediction_nodes = []
    for sensor_key, sensor_config in config['sensors'].items():
        if sensor_key in sensor_port_mapping:
        # print(f"Sensor key: {sensor_key}, Serial port: {serial_port}")
            if isinstance(sensor_config, dict) and sensor_config.get('active', False) and sensor_config.get('type', '') == "SCPS":
                sensor_prediction_nodes = build_prediction_nodes(config, sensor_key, sensor_config)
                prediction_nodes.extend(sensor_prediction_nodes)

    tracker_nodes = []
    for sensor_key, sensor_config in config['sensors'].items():
        if sensor_key in sensor_port_mapping:
        # print(f"Sensor key: {sensor_key}, Serial port: {serial_port}")
            if isinstance(sensor_config, dict) and sensor_config.get('active', False) and sensor_config.get('type', '') == "SCPS":
                sensor_prediction_nodes = build_tracker_nodes(config, sensor_key, sensor_config)
                tracker_nodes.extend(sensor_prediction_nodes)
    
    viz_nodes = build_viz_nodes(config)
    
    camera_nodes = build_cameras_nodes(config)
    joint_relay_nodes = build_joint_relay_nodes(config)
    udp_listener_nodes = build_udp_listener_nodes(config)
    timer_period = 0.0
    timer_period_delay = 0.0

    # Build launch actions list
    launch_actions = []
    for robot_node in robot_nodes:
        launch_actions.append(TimerAction(period=timer_period, actions=[robot_node]))
        timer_period += timer_period_delay

    # Add visualization nodes
    for viz_node in viz_nodes:
        launch_actions.append(TimerAction(period=timer_period, actions=[viz_node]))
        timer_period += timer_period_delay

    # Add camera nodes with delays
    #for camera_node in camera_nodes:
    #    launch_actions.append(TimerAction(period=timer_period, actions=[camera_node]))
    #    timer_period += timer_period_delay

    # Add controller nodes with delays
    #for controller_node in controller_nodes:
      #  launch_actions.append(TimerAction(period=timer_period, actions=[controller_node]))
     #   timer_period += timer_period_delay

    # Add sensor nodes with delays
    #for sensor_node in sensor_nodes:
    #    launch_actions.append(TimerAction(period=timer_period, actions=[sensor_node]))
    #    timer_period += timer_period_delay
    
    # Add prediction nodes with delays
    #for prediction_node in prediction_nodes:
    #    launch_actions.append(TimerAction(period=timer_period, actions=[prediction_node]))
    #    timer_period += timer_period_delay

    # Add joint relay nodes with delays
    for joint_relay_node in joint_relay_nodes:
        launch_actions.append(TimerAction(period =timer_period, actions=[joint_relay_node]))
        timer_period += timer_period_delay

    # Add tracker nodes with delays
    for tracker_node in tracker_nodes:
        launch_actions.append(TimerAction(period=timer_period, actions=[tracker_node]))
        timer_period += timer_period_delay

    # Add udp listener nodes with delays
    #for udp_listener_node in udp_listener_nodes:
    #    launch_actions.append(TimerAction(period=timer_period+20.0, actions=[udp_listener_node]))
    #   timer_period += timer_period_delay

    #print("Running ros1_ros2_bridge")
    #ros1_ros2_bridge()
    
    return launch_actions

def generate_launch_description():

    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value='full_body_avoidance.yaml',
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

def scan_acm_ports_and_get_serial_numbers():
    """
    Scan available ACM ports, connect to each, read serial number, and return mapping.
    Returns dict mapping serial_number -> port_path
    """
    serial_port_mapping = {}
    
    # Find all available ACM ports
    acm_ports = glob.glob('/dev/ttyACM*')
    
    for port in acm_ports:
        # print(f"Trying to connect to {port}")
        try:
            # Try to open serial connection
            ser = serial.Serial(port, 9600, timeout=2.0)
            time.sleep(1.0)  # Give device time to initialize
            
            # Clear any existing data
            ser.flushInput()
            ser.flushOutput()
            
            # Try to read serial number (assuming device sends it as first line or responds to query)
            # You may need to adjust this based on your device's protocol
            try:
                # Option 1: Device automatically sends serial number
                # Wait for up to 2 seconds for a response
                start_time = time.time()
                while time.time() - start_time < 2.0:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').strip()
                        print(f"Received response from {port}: {line}")
                        if line:
                            # Take only the first element from comma-delimited string
                            first_element = line.split(',')[0].strip()
                            # print(f"First element: {first_element}")
                            if first_element and 'S' in first_element and first_element.replace('S', '').isdigit():
                                serial_num = first_element
                                serial_port_mapping[serial_num] = port
                                print(f"Found sensor {serial_num} on {port}")
                                break
                        break
                    time.sleep(0.1)  # Check every 100ms
                else:
                    print(f"No response from {port} within 2 seconds")
                        
            except Exception as e:
                print(f"Error reading serial number from {port}: {e}")
                
            ser.close()
            
        except serial.SerialException as e:
            print(f"Could not open {port}: {e}")
            continue
        except Exception as e:
            print(f"Unexpected error with {port}: {e}")
            continue
    
    return serial_port_mapping


def map_serial_numbers_to_sensors(config):
    """
    Map serial numbers from ACM ports to sensor configurations.
    Returns dict mapping sensor_key -> port_path
    """
    # Get available ports and their serial numbers
    port_mapping = scan_acm_ports_and_get_serial_numbers()
    
    # Create mapping from sensor config to port
    sensor_port_mapping = {}
    
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('active', False):
            # Check if sensor has a serial number defined
            expected_serial = sensor_config.get('SN', None)
            
            if expected_serial and expected_serial in port_mapping:
                sensor_port_mapping[sensor_key] = port_mapping[expected_serial]
                print(f"Mapped {sensor_key} (SN: {expected_serial}) to {port_mapping[expected_serial]}")
            else:
                print(f"Warning: No port found for {sensor_key} with SN: {expected_serial}")
                # Fallback to default port if specified
                if 'port' in sensor_config:
                    if sensor_config['port'] == '':
                        print(f"No fallback port set for {sensor_key}")
                        continue
                    else:
                        sensor_port_mapping[sensor_key] = sensor_config['port']
                        print(f"Using fallback port {sensor_config['port']} for {sensor_key}")
    
    return sensor_port_mapping
