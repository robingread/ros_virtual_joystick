#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    num_pads = LaunchConfiguration("num_pads")
    joy_topic = LaunchConfiguration("joy_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_pads",
                default_value="2",
                description="Number of pads use. Acceptable values are [1, 2].",
                choices=["1", "2"],
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="Topic to publish the joy data to.",
            ),
            Node(
                package="ros_virtual_joystick",
                executable="ros_virtual_joystick_node",
                name="ros_virtual_joystick",
                output="screen",
                parameters=[{"num_pads": num_pads}, {"joy_topic": joy_topic}],
            ),
        ]
    )
