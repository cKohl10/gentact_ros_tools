import argparse
import os

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import yaml
from ament_index_python.packages import (
    get_package_share_directory,  # using this one because it returns a string directly
)
from launch_ros.substitutions import FindPackageShare
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def load_config(config_file_name):
    # provide config: config:={name}.yaml

    package_share = get_package_share_directory("gentact_ros_tools_hybrid")
    config_file = os.path.join(package_share, "config", config_file_name)

    with open(config_file, "r") as file:
        config = yaml.safe_load(file)

    return config


def build_topics(config):
    topics = []
    try:
        for link_key, sensor_config in config["sensors"].items():
            if isinstance(sensor_config, dict) and sensor_config.get("active", False):
                if sensor_config.get("type", "") == "SPAD":
                    for sensor_key in range(sensor_config.get("num_sensors", 0)):
                        link_num = [
                            int(char) for char in list(link_key) if char.isdigit()
                        ][0]

                        if link_num == 5:
                            if "back" in link_key:
                                topics.append(
                                    f"/link{link_num}_back_sensor_{sensor_key}"
                                )
                            else:
                                topics.append(
                                    f"/link{link_num}_front_sensor_{sensor_key}"
                                )
                        else:
                            topics.append(f"/link{link_num}_sensor_{sensor_key}")
                        print(f"Adding topic: {topics[-1]}")
    except KeyError:
        raise KeyError("No sensors in config file")
    return topics


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__("pointcloud_merger")

        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--config",
            type=str,
            default="hybrid.yaml",
            help="Path to specified config file",
        )
        args = parser.parse_args()
        self.config = load_config(args.config)

        topics = build_topics(self.config)  # your topics here

        self.clouds = {}
        self.publisher = self.create_publisher(PointCloud2, "/cloud_merged", 10)

        for topic in topics:
            self.create_subscription(
                PointCloud2, topic, lambda msg, t=topic: self.callback(msg, t), 10
            )

    def callback(self, msg, topic):
        self.clouds[topic] = msg
        if len(self.clouds) < 27:  # wait until we have all clouds
            return

        # Concatenate all point arrays
        all_points = []
        frame_id = None
        stamp = None
        for cloud_msg in self.clouds.values():
            pts = list(pc2.read_points(cloud_msg, skip_nans=True))
            all_points.extend(pts)
            frame_id = cloud_msg.header.frame_id
            stamp = cloud_msg.header.stamp

        # Publish merged cloud
        merged = pc2.create_cloud(cloud_msg.header, cloud_msg.fields, all_points)
        merged.header.frame_id = frame_id  # assumes all clouds in same frame
        self.publisher.publish(merged)


def main():
    rclpy.init()
    rclpy.spin(PointCloudMerger())


if __name__ == "__main__":
    main()
