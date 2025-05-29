#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="ros_virtual_joystick",
                executable="ros_virtual_joystick_node",
                name="ros_virtual_joystick",
                output="screen",
            ),
        ]
    )
