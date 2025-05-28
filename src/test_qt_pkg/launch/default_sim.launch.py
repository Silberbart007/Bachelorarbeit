from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():

    stage_launch_path = os.path.join(
        FindPackageShare('stage_ros2').find('stage_ros2'),
        'launch',
        'stage.launch.py'
    )

    # Pfad zum bringup-Launchfile
    bringup_launch_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    map_yaml_file = PathJoinSubstitution([
        FindPackageShare("stage_ros2"),
        "world",
        "bitmaps",
        "cave_map.yaml"
    ])

    # Eigene Nodes
    camera_node = Node(
        package='test_qt_pkg',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )
    gui_main_node = Node(
        package='test_qt_pkg',
        executable='gui_main',
        name='gui_main',
        output='screen'
    )

    return LaunchDescription([
        # Nav2 bringup inkludieren
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
            launch_arguments={
                'map': map_yaml_file,   # falls du MapServer willst
                'use_sim_time': 'true',
                'autostart': 'true',
                # 'params_file': '/path/to/nav2_params.yaml'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage_launch_path),
            launch_arguments={
                'use_sim_time' : 'true'
            }.items()
        ),

        # Eigene Nodes starten
        camera_node,
        gui_main_node,
    ])
