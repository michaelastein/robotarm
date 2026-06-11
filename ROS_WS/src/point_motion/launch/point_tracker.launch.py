from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    moveit_pkg = get_package_share_directory('adi_robotarm_moveit_config')

    # ---------------- MOVEIT + RVIZ ----------------
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'demo.launch.py')
        )
    )

    # ---------------- VISION NODE ----------------
    vision_node = Node(
        package='vision_tracker',
        executable='hotspot_tracker',
        name='vision_tracker',
        output='screen'
        
    )

    # ---------------- PATH NODE ----------------
    path_node = Node(
        package='point_motion',
        executable='point_motion_path',   # must match ROS2 executable name
        name='point_motion_path',
        output='screen'
    )

    return LaunchDescription([
        moveit_demo,
        vision_node,
        path_node
    ])
