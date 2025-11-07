from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    camera_left = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='left_camera',
        name='left_cam',
        parameters=[{'serial_no': '836612071883'}],
    )

    camera_right = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='right_camera',
        name='right_cam',
        parameters=[{'serial_no': '832112072986'}],
    )

    webcam_node = Node(
        package='camera_tools',
        executable='basic_camera_node',
        name='webcam',
        arguments=['-c', '0']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # camera_left,
        # camera_right,
        webcam_node,
    ])
