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
    
    trajectory_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "trajectory_position_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    return LaunchDescription([
        joints_state_broadcaster_spawner,
        trajectory_position_controller_spawner
    ])
