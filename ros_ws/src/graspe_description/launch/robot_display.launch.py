import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkgPath = FindPackageShare(package='graspe_description').find('graspe_description')
    urdfModelPath = os.path.join(pkgPath, 'urdf', 'robot.urdf')
    rvizConfigPath = os.path.join(pkgPath, 'rviz', 'display_config.rviz')

    # Ler o conteúdo do arquivo URDF
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    # Definir os parâmetros do nó
    params = {'robot_description': robot_desc}

    # Definir o nó do estado do robô
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Definir o nó do publicador de estado das juntas
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params]
    )

    # Definir o nó do publicador de estado das juntas com interface gráfica
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Definir o nó do RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag para iniciar o joint_state_publisher_gui'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
