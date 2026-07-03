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
                executable="hotspot_servo.py",
                name="hotspot_servo",
                output="screen",
            ),

            Node(
                package="robotarm_software",
                executable="hotspot_web_stream.py",
                name="hotspot_web_stream",
                output="screen",
            ),
        ]
    )
