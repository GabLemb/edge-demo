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

    # Avvio Gazebo (server + client) con il nostro world
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

    # 3 Observer in posizioni distanziate dentro l'arena 6x6
    robots = [
        {'name': 'observer_1', 'x': -2.0, 'y': -2.0, 'yaw': '0.0'},
        {'name': 'observer_2', 'x':  0.0, 'y':  2.0, 'yaw': '0.0'},
        {'name': 'observer_3', 'x':  2.0, 'y': -2.0, 'yaw': '0.0'},
    ]

    # Ritardo lo spawn di qualche secondo per dare tempo a Gazebo di partire
    for i, r in enumerate(robots):
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
                    '-Y', r['yaw'],
                ],
                output='screen',
            )],
        ))

    return ld