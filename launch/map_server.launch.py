# /mnt/nvme/ros2_ws/src/swarm_arena/launch/map_server.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_arena')

    map_yaml = os.path.join(pkg_swarm, 'maps', 'arena_map.yaml')

    ld = LaunchDescription()

    # Map server: pubblica la mappa statica su /map
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
            'frame_id': 'map',
            'topic_name': '/map',
        }],
    ))

    # Lifecycle manager: attiva map_server automaticamente
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }],
    ))

    return ld