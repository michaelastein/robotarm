import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


# =========================
# Helper: load YAML properly
# =========================
def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

    # =========================
    # Package paths
    # =========================
    moveit_pkg = get_package_share_directory("robot_moveit_config")

    servo_yaml = os.path.join(moveit_pkg, "config", "servo_parameters.yaml")

    # =========================
    # Core MoveIt launch files
    # =========================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "rsp.launch.py")
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "move_group.launch.py")
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
    # MoveIt Servo (FIXED)
    # =========================
    servo_node = Node(
    	package="moveit_servo",
    	executable="servo_node",
    	name="moveit_servo",
    	output="screen",
	parameters=[load_yaml(servo_yaml)]
     )
    # =========================
    # Custom nodes
    # =========================
    hotspot_tracker = Node(
        package="vision_tracker",
        executable="hotspot_tracker",
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
    # Launch description
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
