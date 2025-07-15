from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='Serial port for sensor data'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    num_sensors_arg = DeclareLaunchArgument(
        'num_sensors',
        default_value='3',
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
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        num_sensors_arg,
        publish_rate_arg,
        sensor_publisher_node,
    ]) 