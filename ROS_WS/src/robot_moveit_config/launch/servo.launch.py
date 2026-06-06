import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():

    pkg = get_package_share_directory("robot_moveit_config")

    # ----------------------------
    # Robot description (URDF via xacro)
    # ----------------------------
    robot_description = {
        "robot_description": Command([
            "xacro ",
            os.path.join(pkg, "config", "visual.urdf.xacro")
        ])
    }

    # ----------------------------
    # Robot State Publisher (REQUIRED FIX)
    # ----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ----------------------------
    # Joint State Publisher (SIMULATION FIX)
    # ----------------------------
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    # ----------------------------
    # MoveIt core
    # ----------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "rsp.launch.py")
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "move_group.launch.py")
        )
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "spawn_controllers.launch.py")
        )
    )

    # ----------------------------
    # Servo node
    # ----------------------------
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            os.path.join(pkg, "config", "servo_parameters.yaml"),
            robot_description
        ],
    )

    # ----------------------------
    # RViz
    # ----------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg, "config", "moveit.rviz"),
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,   # <-- IMPORTANT FIX
        rsp,
        move_group,
        controllers,
        servo_node,
        rviz,
    ])
