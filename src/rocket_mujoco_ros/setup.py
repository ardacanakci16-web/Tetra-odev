import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rocket_mujoco_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'assets'), glob('rocket_mujoco_ros/assets/*.xml')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arda',
    maintainer_email='ardacanakci16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
'mujoco_node = rocket_mujoco_ros.mujoco_node:main',
'gazebo_node = rocket_mujoco_ros.gazebo_node:main',
'sensor_node = rocket_mujoco_ros.sensor_node:main',
'rl_agent_node = rocket_mujoco_ros.rl_agent_node:main',
'action_safety_filter_node = rocket_mujoco_ros.action_safety_filter_node:main',
'observation_builder_node = rocket_mujoco_ros.observation_builder_node:main',
'command_node = rocket_mujoco_ros.command_node:main',

        ],
    },
)
