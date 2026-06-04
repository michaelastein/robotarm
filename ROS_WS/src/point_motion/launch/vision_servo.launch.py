from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
        executable='hotspot_tracker',   # MUST match setup.py entry_point
        name='vision_tracker',
        output='screen',
        parameters=[{
            'image_width': 1280,
            'image_height': 720
        }]
    )

    # ---------------- SERVO NODE (C++) ----------------
    servo_node = Node(
        package='point_motion',
        executable='point_following_2d',
        name='servo_controller',
        output='screen',
        parameters=[{
            'image_width': 1280,
            'image_height': 720
        }]
    )

    # Delay servo slightly so MoveIt + controllers are ready
    delayed_servo = TimerAction(
        period=3.0,
        actions=[servo_node]
    )

    return LaunchDescription([
        moveit_demo,
        vision_node,
        delayed_servo
    ])
