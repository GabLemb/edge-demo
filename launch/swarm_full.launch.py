# /mnt/nvme/ros2_ws/src/swarm_arena/launch/swarm_full.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_arena')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(pkg_swarm, 'worlds', 'arena.world')
    sdf_model = os.path.join(
        pkg_tb3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf'
    )

    ld = LaunchDescription()

    # Gazebo
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    ))

    robots = [
        {'name': 'observer_1', 'x': -2.0, 'y': -2.0, 'turn':  1},
        {'name': 'observer_2', 'x':  0.0, 'y':  2.0, 'turn': -1},
        {'name': 'observer_3', 'x':  2.0, 'y': -2.0, 'turn':  1},
    ]

    # UN UNICO loop che configura tutti i nodi per ogni osservatore
    for i, r in enumerate(robots):
        # Spawn del robot in Gazebo
        ld.add_action(TimerAction(
            period=float(5 + i * 2),
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{r["name"]}',
                arguments=[
                    '-entity', r['name'],
                    '-file', sdf_model,
                    '-robot_namespace', r['name'],
                    '-x', str(r['x']),
                    '-y', str(r['y']),
                    '-z', '0.01',
                ],
                output='screen',
            )],
        ))

        # Random patrol
        ld.add_action(TimerAction(
            period=float(12 + i * 2),
            actions=[Node(
                package='swarm_arena',
                executable='random_patrol',
                namespace=r['name'],
                name='random_patrol',
                output='screen',
                parameters=[{
                    'linear_speed': 0.12,
                    'angular_speed': 0.6,
                    'safe_distance': 0.5,
                    'front_cone_deg': 60.0,
                    'preferred_turn': r['turn'],
                }],
            )],
        ))

        # Target tracker
        ld.add_action(TimerAction(
            period=float(13 + i * 2),
            actions=[Node(
                package='swarm_arena',
                executable='target_tracker',
                namespace=r['name'],
                name='target_tracker',
                output='screen',
                parameters=[{
                    'observer_name': r['name'],
                    'detection_radius': 2.5,
                    'publish_rate': 5.0,
                }],
            )],
        ))

        # Data layer (mock Myrmic)
        ld.add_action(TimerAction(
            period=float(14 + i * 2),
            actions=[Node(
                package='swarm_arena',
                executable='data_layer',
                namespace=r['name'],
                name='data_layer',
                output='screen',
                parameters=[{
                    'observer_name': r['name'],
                    'http_port': 5001 + i,
                    'storage_dir': '/mnt/nvme/myrmic_mock',
                }],
            )],
        ))

    # Target mover (singolo, fuori namespace)
    ld.add_action(TimerAction(
        period=20.0,
        actions=[Node(
            package='swarm_arena',
            executable='target_mover',
            name='target_mover',
            output='screen',
        )],
    ))

    return ld