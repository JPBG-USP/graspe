from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    joints_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    position_controler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controler",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    return LaunchDescription(
        joints_state_broadcaster_spawner,
        position_controler_spawner
    )