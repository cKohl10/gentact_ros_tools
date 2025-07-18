from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = 'calibration'
    
    # Generate URDF from xacro file (or read URDF file)
    sensor_urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'link5_2.xacro'])
    sensor_description = ParameterValue(Command(['xacro ', sensor_urdf_file, f' namespace:={namespace}_link5']), value_type=str)

    robot_urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'fr3_plate.urdf'])
    robot_description = ParameterValue(Command(['cat ', robot_urdf_file]), value_type=str)

    # Create nodes
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    sensor_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sensor_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': sensor_description}],
        remappings=[('/robot_description', '/sensor_description')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='fr3_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    # Add joint state publisher for FR3 robot
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='fr3_joint_state_publisher',
        output='screen',
    )

    sensor_st_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_static_transform_publisher',
        output='screen',
        arguments=['0.53', '-0.01', '-0.11', '1.57079632679', '0', '1.7', 'map', f'{namespace}_link5/base_link']
    )

    robot_st_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        foxglove_bridge_node,
        TimerAction(period=1.0, actions=[robot_state_publisher_node]),  # 1 second delay
        TimerAction(period=2.0, actions=[sensor_state_publisher_node]),  # 2 second delay
        # TimerAction(period=3.0, actions=[joint_state_publisher_node]),  # 3 second delay
        TimerAction(period=3.0, actions=[robot_st_base_node]),  # 4 second delay
        TimerAction(period=5.0, actions=[sensor_st_base_node]),  # 5 second delay
    ])