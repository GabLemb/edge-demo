# /mnt/nvme/ros2_ws/src/swarm_arena/launch/swarm_tf.launch.py
import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_arena')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    world = os.path.join(pkg_swarm, 'worlds', 'arena.world')
    sdf_template = os.path.join(
        pkg_swarm, 'models', 'turtlebot3_burger', 'model.sdf'
    )
    urdf_file = os.path.join(
        pkg_tb3_desc, 'urdf', 'turtlebot3_burger.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    with open(sdf_template, 'r') as f:
        sdf_template_str = f.read()

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    ))

    robots = [
        {'name': 'observer_1', 'x': -2.0, 'y': -2.0},
        {'name': 'observer_2', 'x':  0.0, 'y':  2.0},
        {'name': 'observer_3', 'x':  2.0, 'y': -2.0},
    ]

    for i, r in enumerate(robots):
        # Genera SDF specifico per questo robot, sostituendo il segnaposto
        sdf_for_robot = sdf_template_str.replace(
            '__FRAME_PREFIX__', f'{r["name"]}/'
        )
        # Salva in un file temporaneo (spawn_entity.py vuole un path)
        tmp = tempfile.NamedTemporaryFile(
            mode='w', suffix=f'_{r["name"]}.sdf', delete=False
        )
        tmp.write(sdf_for_robot)
        tmp.close()

        # robot_state_publisher con frame_prefix
        ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=r['name'],
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'frame_prefix': f'{r["name"]}/',
            }],
            output='screen',
        ))

        # Spawn ritardato con SDF custom
        ld.add_action(TimerAction(
            period=float(5 + i * 2),
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{r["name"]}',
                arguments=[
                    '-entity', r['name'],
                    '-file', tmp.name,
                    '-robot_namespace', r['name'],
                    '-x', str(r['x']),
                    '-y', str(r['y']),
                    '-z', '0.01',
                ],
                output='screen',
            )],
        ))

    return ld