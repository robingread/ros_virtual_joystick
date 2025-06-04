#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    num_pads = LaunchConfiguration("num_pads")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_pads",
                default_value="2",
                description="Number of pads use. Acceptable values are [1, 2].",
                choices=["1", "2"],
            ),
            Node(
                package="ros_virtual_joystick",
                executable="ros_virtual_joystick_node",
                name="ros_virtual_joystick",
                output="screen",
                parameters=[{"num_pads": num_pads}],
            ),
        ]
    )
