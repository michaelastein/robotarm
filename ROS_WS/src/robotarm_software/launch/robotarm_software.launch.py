import os

import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo


def generate_launch_description():
    pkg_path = get_package_share_directory("robotarm_software")

    urdf_file = os.path.join(
        pkg_path,
        "urdf",
        "robotarm.urdf",
    )

    controllers_yaml = os.path.join(
        pkg_path,
        "config",
        "ros2_controllers.yaml",
    )

    with open(urdf_file, "r") as f:
        robot_description = {
            "robot_description": f.read()
        }

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
        ],
    )

    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml,
        ],
        output="screen",
    )

    joint_state_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    servo_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "servo_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    static_tf2_broadcaster_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf2_broadcaster",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "world",
            "base_link",
        ],
        output="screen",
    )

    safety_node = launch_ros.actions.Node(
        package="robotarm_software",
        executable="safety_node.py",
        name="safety_node",
        output="screen",
    )

    safety_supervisor_node = launch_ros.actions.Node(
        package="robotarm_software",
        executable="safety_supervisor.py",
        name="safety_supervisor",
        output="screen",
    )

    cam0_params = {
        "camera": 0,
        "width": 320,
        "height": 240,
        "AfMode": 2,
        "AwbEnable": True,
        "AeEnable": True,
    }

    cam0_node = launch_ros.actions.Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        namespace="cam0",
        parameters=[
            cam0_params,
        ],
        output="screen",
    )

    # Optional: automatically start custom controller.
    # During testing, I recommend starting it manually first.
    #
    # tool_position_velocity_control_node = launch_ros.actions.Node(
    #     package="robotarm_software",
    #     executable="tool_position_velocity_control.py",
    #     name="tool_position_velocity_control",
    #     output="screen",
    # )

    startup_message = LogInfo(
        msg=(
            "\n"
            "========================================\n"
            " robotarm_software started\n"
            "----------------------------------------\n"
            " MoveIt Servo is NOT launched.\n"
            " Active low-level command topic:\n"
            "   /servo_controller/commands\n"
            "\n"
            " Expected publishers while controller is running:\n"
            "   safety_supervisor\n"
            "   tool_position_velocity_control\n"
            "\n"
            " Check with:\n"
            "   ros2 topic info /servo_controller/commands -v\n"
            "========================================\n"
        )
    )

    return launch.LaunchDescription(
        [
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_spawner,
            servo_controller_spawner,
            static_tf2_broadcaster_node,
            safety_node,
            safety_supervisor_node,
            cam0_node,
            startup_message,
            # tool_position_velocity_control_node,
        ]
    )
