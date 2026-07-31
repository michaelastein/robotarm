from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "robotarm",
            package_name="robotarm_moveit_config",
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_pipelines()
        .to_moveit_configs()
    )

    hotspot_planner = Node(
        package="robotarm_path_planning",
        executable="hotspot_path_planner",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
            },
        ],
    )

    return LaunchDescription([
        hotspot_planner,
    ])
