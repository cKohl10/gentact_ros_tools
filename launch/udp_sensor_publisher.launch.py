from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gentact_ros_tools',
            executable='udp_sensor_publisher',
            name='udp_sensor_publisher',
            output='screen',
            parameters=[{
                'udp_port': 8888,
                'buffer_size': 1024,
                'timeout_seconds': 5.0,
                'max_devices': 10  # Support up to 10 devices
            }]
        )
    ]) 