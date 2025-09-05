#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robin_ros2_cpp",
                namespace="robin_ros2",
                executable="robin_platform_interface_node",
                name="robin_platform_interface_node",
                remappings=[
                    # ("/from", "/to"),
                ],
                parameters=[],
            ),
            Node(
                package="robin_ros2_py",
                namespace="robin_ros2",
                executable="dashboard_node",
                name="dashboard_node",
                remappings=[
                    # ("/from", "/to"),
                ],
                parameters=[],
            ),
        ]
    )
