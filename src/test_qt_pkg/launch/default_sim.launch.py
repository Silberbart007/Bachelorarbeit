from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    stage_launch_path = os.path.join(
        FindPackageShare('stage_ros2').find('stage_ros2'),
        'launch',
        'stage.launch.py'
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/user/vtulkinm0/Ros_workspaces/Bachelorarbeit/src/stage_ros2/world/bitmaps/cave_map.yaml'}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                '/home/user/vtulkinm0/Ros_workspaces/Bachelorarbeit/src/test_qt_pkg/config/amcl.yaml'
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage_launch_path)
        ),
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
