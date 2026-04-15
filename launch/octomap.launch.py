from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="octomap_server2",
                executable="octomap_server_node",
                name="octomap_server",
                parameters=[
                    {
                        "resolution": 0.05,  # voxel size in meters
                        "frame_id": "map",  # fixed frame
                        "sensor_model.max_range": 5.0,
                    }
                ],
                remappings=[("cloud_in", "/combined_pc")],
            )
        ]
    )
