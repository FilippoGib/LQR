from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('lqr'),
        'config',
        'lqr.yaml'
    )

    lqr_node = Node(
        name='lqr',
        package='lqr',
        executable='lqr_node',
        parameters=[config],
        output="screen"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'
            ),
            lqr_node
        ]
    )