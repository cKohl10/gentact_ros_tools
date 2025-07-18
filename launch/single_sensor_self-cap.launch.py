from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_urdf_file = PathJoinSubstitution([
        FindPackageShare('gentact_ros_tools'),
        'urdf',
        'fr3_link6.urdf'
    ])
    robot_description = ParameterValue(Command(['cat ', robot_urdf_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='fr3_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='fr3_joint_state_publisher',
        output='screen'
    )

    robot_st_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    # Sensor     # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for sensor data'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    num_sensors_arg = DeclareLaunchArgument(
        'num_sensors',
        default_value='6',
        description='Number of sensors'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Publishing rate in Hz'
    )

    # Create the sensor publisher node
    sensor_publisher_node = Node(
        package='gentact_ros_tools',
        executable='sensor_publisher',
        name='sensor_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'num_sensors': LaunchConfiguration('num_sensors'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='cam_pub',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        serial_port_arg,
        baud_rate_arg,
        num_sensors_arg,
        publish_rate_arg,
        TimerAction(period=0.0, actions=[robot_st_base_node]),
        TimerAction(period=1.0, actions=[robot_state_publisher_node]),
        TimerAction(period=1.5, actions=[joint_state_publisher_node]),
        TimerAction(period=2.0, actions=[rviz_node]),
        # TimerAction(period=5.0, actions=[camera_node]),
        # TimerAction(period=6.0, actions=[sensor_publisher_node]),

    ])
