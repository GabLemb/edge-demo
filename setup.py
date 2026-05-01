from setuptools import setup
import os
from glob import glob

package_name = 'swarm_arena'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rr',
    maintainer_email='lembogabriella@gmail.com',
    description='Swarm arena for Myrmic demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'random_patrol = swarm_arena.random_patrol_node:main',
            'target_mover = swarm_arena.target_mover_node:main',
            'target_tracker = swarm_arena.target_tracker_node:main',
            'data_layer = swarm_arena.data_layer_node:main',
        ],
    },
)