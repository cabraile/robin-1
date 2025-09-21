#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robin_ros2_py",
                namespace="robin_ros2",
                executable="platform_client_node",
                name="platform_client_node",
                remappings=[
                    # ("/from", "/to"),
                ],
                parameters=[],
            ),
        ]
    )
