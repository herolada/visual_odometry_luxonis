import os
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def launch_setup(*args, **kwargs):
    share_dir = get_package_share_directory('visual_odometry')
    list_params = [os.path.join(share_dir, 'config', 'config.yaml')]

    remappings = [ #('odom', 'gps_odom')
    ]

    return [
        Node(
            package='visual_odometry',
            executable='visual_odometry.py',
            name='visual_odometry',
            output='screen',
            parameters=list_params,
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        launch_setup()
    )
