from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_share = FindPackageShare(
        "robotarm_moveit_config"
    ).find("robotarm_moveit_config")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(moveit_share)
                / "launch"
                / "move_group.launch.py"
            )
        )
    )

    joint_states_relay = Node(
        package="robotarm_software",
        executable="joint_states_relay.py",
        name="joint_states_relay",
        output="screen",
    )

    hotspot_detector = Node(
        package="robotarm_software",
        executable="hotspot_detector.py",
        name="hotspot_detector",
        output="screen",
        arguments=[
            "--mode",
            mode,
        ],
    )

    hotspot_path_planner = Node(
        package="robotarm_software",
        executable="hotspot_path_planner",
        name="hotspot_path_planner",
        output="screen",
    )

    delayed_path_planner = TimerAction(
        period=3.0,
        actions=[
            hotspot_path_planner,
        ],
    )

    return LaunchDescription([
        move_group,
        joint_states_relay,
        hotspot_detector,
        delayed_path_planner,
    ])
