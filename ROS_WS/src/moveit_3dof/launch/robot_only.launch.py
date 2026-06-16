import os

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("moveit_3dof")

    moveit_config = (
        MoveItConfigsBuilder(
            "visual",
            package_name="moveit_3dof"
        )
        # THIS is the ONLY correct way in your version
        .joint_limits("config/joint_limits.yaml")
        .to_moveit_configs()
    )

    demo = generate_demo_launch(moveit_config)

    safety_node = Node(
        package="moveit_3dof",
        executable="safety_node.py",
        name="safety_node",
        output="screen",
    )

    safety_supervisor = Node(
        package="moveit_3dof",
        executable="safety_supervisor.py",
        name="safety_supervisor",
        output="screen",
    )


    hotspot_visual_servo = Node(
        package="moveit_3dof",
        executable="hotspot_visual_servo.py",
        name="hotspot_visual_servo",
        output="screen",
    )

    return LaunchDescription(
        list(demo.entities)
        + [
            safety_node,
            safety_supervisor,
            hotspot_visual_servo,
        ]
    )
