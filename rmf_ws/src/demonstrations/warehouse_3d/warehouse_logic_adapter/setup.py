from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_logic_adapter'

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
    description='Universal integration adapter connecting external autonomous warehouse logic to ROS 2 and Gazebo Sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logic_adapter_node = warehouse_logic_adapter.logic_adapter_node:main',
            'mock_controller = warehouse_logic_adapter.mock_controller:main',
        ],
    },
)
