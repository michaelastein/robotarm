import os

import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_path = get_package_share_directory("moveit_3dof")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="robotarm",
            package_name="moveit_3dof",
        )
        .robot_description(file_path="urdf/robotarm.urdf")
        .robot_description_semantic(file_path="config/visual.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node",
        default="false",
    )

    servo_yaml = os.path.join(
        pkg_path,
        "config",
        "servo_parameters.yaml",
    )

    controllers_yaml = os.path.join(
        pkg_path,
        "config",
        "ros2_controllers.yaml",
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
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

    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_yaml,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[
                    {
                        "frame_id": "world",
                        "child_frame_id": "base_link",
                    }
                ],
            ),
        ],
        output="screen",
    )

    safety_node = launch_ros.actions.Node(
        package="moveit_3dof",
        executable="safety_node.py",
        name="safety_node",
        output="screen",
    )

    safety_supervisor_node = launch_ros.actions.Node(
        package="moveit_3dof",
        executable="safety_supervisor.py",
        name="safety_supervisor",
        output="screen",
    )

#    hotspot_visual_servo_node = launch_ros.actions.Node(
 #       package="moveit_3dof",
  #      executable="hotspot_visual_servo.py",
   #     name="hotspot_visual_servo",
    #    output="screen",
   # )

    return launch.LaunchDescription(
        [
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_spawner,
            servo_controller_spawner,
            container,
            safety_node,
            safety_supervisor_node,
    #        hotspot_visual_servo_node,
        ]
    )
