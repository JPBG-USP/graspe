import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the argument 'gazebo'
    gazebo_arg = DeclareLaunchArgument(
        'gazebo', default_value='false', description='Whether to launch gazebo visualization or not'
    )

    # Define the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('graspe_description'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('gazebo'))
    )

    # Define the controller launch file
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('graspe_controller'),
                'launch',
                'controller.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('gazebo'))
    )

    # Define the RViz display launch file
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('graspe_description'),
                'launch',
                'display.launch.py'
            )
        ),
        condition=UnlessCondition(LaunchConfiguration('gazebo'))
    )

    return LaunchDescription([
        gazebo_arg,
        gazebo_launch,
        controller_launch,
        display_launch
    ])