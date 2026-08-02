from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('apriltag_ar'),
        'config',
        'camera.yaml'
    )

    return LaunchDescription([
        Node(
            package='apriltag_ar',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='apriltag_ar',
            executable='apriltag_detector',
            name='apriltag_detector',
            output='screen'
        )
    ])
