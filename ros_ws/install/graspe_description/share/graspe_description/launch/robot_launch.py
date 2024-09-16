from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node para carregar o URDF do robô
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'graspe_description': open(
                    [FindPath('urdf/my_robot.urdf')].read()
                )
            }]
        ),
        # Node para iniciar o RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', [FindPath('rviz/default.rviz')]  # Caminho opcional para um arquivo de configuração RViz
            ]
        ),
    ])
