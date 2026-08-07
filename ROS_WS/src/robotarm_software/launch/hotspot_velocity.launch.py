#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [    
            DeclareLaunchArgument(
                "mode",
                default_value="led",
                description="Hotspot detector mode",
            )
            Node(
                package="robotarm_software",
                executable="hotspot_detector.py",
                name="hotspot_detector",
                output="screen",
                arguments=[
                    "--mode",
                    mode,
                ],
            ),

            Node(
                package="robotarm_software",
                executable="hotspot_velocity.py",
                name="hotspot_velocity",
                output="screen",
            ),

        ]
    )
