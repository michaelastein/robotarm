#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robotarm_software",
                executable="hotspot_detector.py",
                name="hotspot_detector",
                output="screen",
            ),

            Node(
                package="robotarm_software",
                executable="hotspot_velocity.py",
                name="hotspot_velocity",
                output="screen",
            ),

        ]
    )
