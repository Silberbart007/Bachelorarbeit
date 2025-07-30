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
        'custom_fun_map.launch.py'
    )

    # Pfad zur custom bringup-yaml (Wo auch Amcl, Mapserver etc. gestartet wird)
    custom_params_path = PathJoinSubstitution([
        FindPackageShare('test_qt_pkg'),
        'config',
        'custom_nav2_params.yaml'
    ])

    # Pfad zum bringup-Launchfile
    bringup_launch_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    map_yaml_file = PathJoinSubstitution([
        FindPackageShare("test_qt_pkg"),
        "config",
        "custom_maps",
        "custom_fun_map.yaml"
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
    tcp_robot_listener_node = Node(
        package='test_qt_pkg',
        executable='tcp_robot_listener_node',
        name='tcp_robot_listener_node',
        output='screen'
    )
    tcp_vel_client_node = Node(
        package='test_qt_pkg',
        executable='tcp_vel_client_node',
        name='tcp_vel_client_node',
        output='screen'
    )

    return LaunchDescription([
        # Nav2 bringup inkludieren
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': custom_params_path
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
        tcp_robot_listener_node,
        tcp_vel_client_node,
    ])
