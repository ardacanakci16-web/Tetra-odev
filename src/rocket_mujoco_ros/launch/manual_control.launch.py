from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mujoco_node = Node(
        package="rocket_mujoco_ros",
        executable="mujoco_node",
        name="mujoco_node",
        output="screen"
    )

    sensor_node = Node(
        package="rocket_mujoco_ros",
        executable="sensor_node",
        name="sensor_node",
        output="screen"
    )

    observation_builder_node = Node(
        package="rocket_mujoco_ros",
        executable="observation_builder_node",
        name="observation_builder_node",
        output="screen"
    )

    action_safety_filter_node = Node(
        package="rocket_mujoco_ros",
        executable="action_safety_filter_node",
        name="action_safety_filter_node",
        output="screen"
    )

    rl_agent_node = Node(
        package="rocket_mujoco_ros",
        executable="rl_agent_node",
        name="rl_agent_node",
        output="screen"
    )

    return LaunchDescription([
        mujoco_node,
        sensor_node,
        observation_builder_node,
        action_safety_filter_node,
        rl_agent_node,
    ])
