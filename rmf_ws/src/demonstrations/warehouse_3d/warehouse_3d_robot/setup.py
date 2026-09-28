from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_3d_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse_sim',
    maintainer_email='warehouse_sim@simulation.local',
    description='Robot spawning, state broadcasting, and TF management for warehouse AMR fleet',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_spawner = warehouse_3d_robot.robot_spawner:main',
            'robot_state_broadcaster = warehouse_3d_robot.robot_state_broadcaster:main',
        ],
    },
)
