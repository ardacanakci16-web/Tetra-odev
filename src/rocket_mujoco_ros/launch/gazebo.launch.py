import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rocket_mujoco_ros')
    default_model_path = os.path.join(pkg_share, 'urdf', 'hopper.urdf')
    
    # We use the standard empty world of Gazebo
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        
        # Start Gazebo server (gzserver)
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Start Gazebo client (gzclient)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
                         'use_sim_time': True}]
        ),

        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "vtvl_hopper", "-z", "11.0"]
        ),

        # Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        # TVC Controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["tvc_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ])
