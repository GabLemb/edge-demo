# /mnt/nvme/ros2_ws/src/swarm_arena/launch/localization.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
import math


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_arena')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    world = os.path.join(pkg_swarm, 'worlds', 'arena.world')
    sdf_template = os.path.join(
        pkg_swarm, 'models', 'turtlebot3_burger', 'model.sdf'
    )
    urdf_file = os.path.join(pkg_tb3_desc, 'urdf', 'turtlebot3_burger.urdf')
    map_yaml = os.path.join(pkg_swarm, 'maps', 'arena_map.yaml')
    amcl_config = os.path.join(pkg_swarm, 'config', 'amcl.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    with open(sdf_template, 'r') as f:
        sdf_template_str = f.read()

    import tempfile

    ld = LaunchDescription()

    # Gazebo headless
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    ))

    # Map server
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

    # I 3 robot
    robots = [
        {'name': 'observer_1', 'x': -2.0, 'y': -2.0, 'yaw': 0.0},
        {'name': 'observer_2', 'x':  0.0, 'y':  2.0, 'yaw': 0.0},
        {'name': 'observer_3', 'x':  2.0, 'y': -2.0, 'yaw': 0.0},
    ]

    # Lista dei nodi che il lifecycle manager dovrà attivare:
    # map_server + 3 amcl
    lifecycle_nodes = ['map_server']

    for i, r in enumerate(robots):
        ns = r['name']
        prefix = f'{ns}/'

        # SDF parametrizzato
        sdf_for_robot = sdf_template_str.replace('__FRAME_PREFIX__', prefix)
        tmp = tempfile.NamedTemporaryFile(
            mode='w', suffix=f'_{ns}.sdf', delete=False
        )
        tmp.write(sdf_for_robot)
        tmp.close()

        # robot_state_publisher
        ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'frame_prefix': prefix,
            }],
            output='screen',
        ))

        # Spawn ritardato
        ld.add_action(TimerAction(
            period=float(5 + i * 2),
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{ns}',
                arguments=[
                    '-entity', ns,
                    '-file', tmp.name,
                    '-robot_namespace', ns,
                    '-x', str(r['x']),
                    '-y', str(r['y']),
                    '-z', '0.01',
                    '-Y', str(r['yaw']),
                ],
                output='screen',
            )],
        ))

        # AMCL per questo robot
        # Override dei frame con il prefisso del robot
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            namespace=ns,
            name='amcl',
            output='screen',
            parameters=[
                amcl_config,
                {
                    'use_sim_time': True,
                    'base_frame_id': f'{prefix}base_footprint',
                    'odom_frame_id': f'{prefix}odom',
                    'global_frame_id': 'map',
                    'scan_topic': f'/{ns}/scan',
                    'initial_pose.x': r['x'],
                    'initial_pose.y': r['y'],
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': r['yaw'],
                },
            ],
            # Remap del topic /map (è globale)
            remappings=[('map', '/map')],
        )
        # AMCL parte dopo lo spawn
        ld.add_action(TimerAction(
            period=float(15 + i * 2),
            actions=[amcl_node],
        ))

        # Nota: AMCL gira in namespace observer_N, quindi il suo nome completo
        # è /observer_N/amcl. Il lifecycle manager lo gestirà con il nome completo.
        lifecycle_nodes.append(f'/{ns}/amcl')

    # Lifecycle manager unico per map_server + 3 AMCL
    # Parte per ultimo, dopo che tutti i nodi sono creati
    ld.add_action(TimerAction(
        period=22.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 0.0,
            }],
        )],
    ))

    # Publisher one-shot della initial pose per ogni AMCL.
    # In Foxy, AMCL ignora 'initial_pose.*' come parametro:
    # bisogna pubblicarla sul topic /observer_N/initialpose dopo l'attivazione.
    for i, r in enumerate(robots):
        ns = r['name']
        # Calcola il quaternione dal yaw
        qz = math.sin(r['yaw'] / 2.0)
        qw = math.cos(r['yaw'] / 2.0)

        msg = (
            f'{{header: {{frame_id: "map"}}, '
            f'pose: {{pose: {{'
            f'position: {{x: {r["x"]}, y: {r["y"]}, z: 0.0}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
            f'}}, '
            f'covariance: ['
            f'0.25, 0, 0, 0, 0, 0, '
            f'0, 0.25, 0, 0, 0, 0, '
            f'0, 0, 0, 0, 0, 0, '
            f'0, 0, 0, 0, 0, 0, '
            f'0, 0, 0, 0, 0, 0, '
            f'0, 0, 0, 0, 0, 0.0685'
            f']}}}}'
        )

        from launch.actions import ExecuteProcess
        ld.add_action(TimerAction(
            period=float(25 + i),
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '-1',
                    '--qos-durability', 'transient_local',
                    f'/{ns}/initialpose',
                    'geometry_msgs/PoseWithCovarianceStamped',
                    msg,
                ],
                output='screen',
            )],
        ))

    return ld