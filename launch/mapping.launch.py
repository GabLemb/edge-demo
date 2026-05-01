# /mnt/nvme/ros2_ws/src/swarm_arena/launch/mapping.launch.py
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
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    world = os.path.join(pkg_swarm, 'worlds', 'arena.world')
    sdf_model = os.path.join(
        pkg_tb3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf'
    )
    urdf_file = os.path.join(
        pkg_tb3_desc, 'urdf', 'turtlebot3_burger.urdf'
    )
    slam_config = os.path.join(pkg_swarm, 'config', 'slam_mapping.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    ld = LaunchDescription()

    # Gazebo SERVER (headless, niente client per evitare crash X11)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    ))

    # Robot state publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='observer_1',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    ))

    # Spawn robot
    ld.add_action(TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_observer_1',
            arguments=[
                '-entity', 'observer_1',
                '-file', sdf_model,
                '-robot_namespace', 'observer_1',
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
            ],
            output='screen',
        )],
    ))

    # SLAM Toolbox
    ld.add_action(TimerAction(
        period=10.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
        )],
    ))

    # Auto-explorer (parte dopo SLAM, gli dà tempo di inizializzarsi)
    ld.add_action(TimerAction(
        period=15.0,
        actions=[Node(
            package='swarm_arena',
            executable='auto_explorer',
            namespace='observer_1',
            name='auto_explorer',
            output='screen',
            parameters=[{
                'linear_speed': 0.15,
                'angular_speed': 0.5,
                'safe_distance': 0.5,
                'explore_duration': 300.0,
                'map_save_path': '/mnt/nvme/ros2_ws/src/swarm_arena/maps/arena_map',
            }],
        )],
    ))

    return ld