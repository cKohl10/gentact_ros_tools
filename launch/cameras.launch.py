from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="cam",
        parameters=[{"serial_no": "909512071540"}],
    )

    camera_right = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="right_camera",
        name="right_cam",
        parameters=[{"serial_no": "832112072986"}],
    )

    webcam_node = Node(
        package="camera_tools",
        executable="basic_camera_node",
        name="webcam",
        parameters=[
            {"camera_index": 0},
            {"publish_rate": 30.0},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # camera,
            # camera_right,
            webcam_node,
        ]
    )
