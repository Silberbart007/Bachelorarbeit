from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_qt_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='test_qt_pkg',
            executable='gui_main',
            name='gui_main',
            output='screen'
        )
    ])
