import os 

  

from launch import LaunchDescription 

from launch_ros.actions import Node 

from moveit_configs_utils import MoveItConfigsBuilder 

from moveit_configs_utils.launches import generate_demo_launch 

from ament_index_python.packages import get_package_share_directory 

  

  

def generate_launch_description(): 

  

    pkg = get_package_share_directory("robot_moveit_config") 

  

    moveit_config = ( 

        MoveItConfigsBuilder( 

            "visual", 

            package_name="robot_moveit_config" 

        ) 

        .to_moveit_configs() 

    ) 

  

    moveit_config_dict = moveit_config.to_dict() 

  

    demo = generate_demo_launch(moveit_config) 

  

    servo_node = Node( 

        package="moveit_servo", 

        executable="servo_node", 

        name="own_servo_node", 

        output="screen", 

        parameters=[ 

            moveit_config_dict, 

            os.path.join(pkg, "config", "servo_parameters.yaml"), 

        ], 

        remappings=[ 

            ("/servo_node/delta_twist_cmds", "/servo_node/delta_twist_cmds"), 

        ], 

    ) 

  

    return LaunchDescription( 

        list(demo.entities) + [servo_node] 

    ) 
