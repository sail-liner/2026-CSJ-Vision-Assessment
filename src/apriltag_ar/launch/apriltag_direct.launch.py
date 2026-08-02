from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ar',
            executable='apriltag_direct',
            name='apriltag_detector',
            output='screen',
            parameters=[{
                'camera_id': 0,
                'camera_frame_id': 'camera_frame',
                'camera_width': 640,
                'camera_height': 480,
                'camera_fps': 30.0,
            }]
        )
    ])
