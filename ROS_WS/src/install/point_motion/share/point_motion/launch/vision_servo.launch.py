import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================
    # Directories
    # =========================
    moveit_pkg = get_package_share_directory("moveit_servo")

    # =========================
    # MoveIt core launch files
    # =========================
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "move_group.launch.py")
        )
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "rsp.launch.py")
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "moveit_rviz.launch.py")
        )
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "spawn_controllers.launch.py")
        )
    )

    # =========================
    # MoveIt Servo Node
    # =========================
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",   # ggf. bei dir: servo_node_main
        name="moveit_servo",
        output="screen",
        parameters=[
            os.path.join(moveit_pkg, "config", "servo_parameters.yaml"),
            os.path.join(moveit_pkg, "config", "ros2_controllers.yaml"),
        ],
    )

    # =========================
    # Custom nodes
    # =========================
    hotspot_tracker = Node(
        package="vision_tracker",
        executable="hotspot_tracker.py",
        name="hotspot_tracker",
        output="screen",
    )

    point_following = Node(
        package="point_motion",
        executable="point_following_2d",
        name="point_following_2d",
        output="screen",
    )

    # =========================
    # Launch Description
    # =========================
    return LaunchDescription([
        rsp,
        move_group,
        controllers,
        rviz,
        servo_node,
        hotspot_tracker,
        point_following,
    ])
