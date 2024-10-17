import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ringbot_bringup'),
        'config',
        'motor_test.yaml'
    )

    return LaunchDescription([
        Node(
            package='dynamixel_motor',
            executable='dynamixel_motor',
            name='dynamixel_motor',
            output='screen',
            parameters=[config]
        )
    ])