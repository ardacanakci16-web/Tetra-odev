import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray, Bool, String

import numpy as np
import json

from rocket_mujoco_ros.rocket_env import FreeJointRocketTerrainLandingEnv
from rocket_mujoco_ros.config import (
    ACTION_SIZE,
    TOPIC_ACTION,
    TOPIC_OBSERVATION,
    TOPIC_REWARD,
    TOPIC_DONE,
    TOPIC_INFO,
    SIM_DT,
)


class MujocoNode(Node):
    def __init__(self):
        super().__init__("mujoco_node")

        self.env = FreeJointRocketTerrainLandingEnv()
        self.obs, self.info = self.env.reset_with_viewer()

        self.action = np.zeros(ACTION_SIZE, dtype=np.float32)

        # --- Publishers ---
        self.observation_pub = self.create_publisher(
            Float32MultiArray,
            TOPIC_OBSERVATION,
            10
        )

        self.reward_pub = self.create_publisher(
            Float32,
            TOPIC_REWARD,
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            TOPIC_DONE,
            10
        )

        self.info_pub = self.create_publisher(
            String,
            TOPIC_INFO,
            10
        )

        self.altitude_pub = self.create_publisher(
            Float32,
            "/rocket/state/altitude",
            10
        )

        self.vertical_velocity_pub = self.create_publisher(
            Float32,
            "/rocket/state/vertical_velocity",
            10
        )

        # --- Subscriber ---
        self.action_sub = self.create_subscription(
            Float32MultiArray,
            TOPIC_ACTION,
            self.action_callback,
            10
        )

        # --- Timer ---
        self.timer = self.create_timer(SIM_DT, self.step_environment)

        self.get_logger().info("MuJoCo Rocket ROS2 node started.")

    def action_callback(self, msg):
        if len(msg.data) != ACTION_SIZE:
            self.get_logger().warn(
                f"Action boyutu yanlış. Beklenen: {ACTION_SIZE}, gelen: {len(msg.data)}"
            )
            return

        self.action = np.array(msg.data, dtype=np.float32)

    def step_environment(self):
        obs, reward, terminated, truncated, info = self.env.step(self.action)

        done = terminated or truncated

        # Observation
        obs_msg = Float32MultiArray()
        obs_msg.data = obs.astype(float).tolist()
        self.observation_pub.publish(obs_msg)

        # Reward
        reward_msg = Float32()
        reward_msg.data = float(reward)
        self.reward_pub.publish(reward_msg)

        # Done
        done_msg = Bool()
        done_msg.data = bool(done)
        self.done_pub.publish(done_msg)

        # Info
        info_msg = String()
        info_msg.data = json.dumps(info)
        self.info_pub.publish(info_msg)

        # State telemetry
        altitude_msg = Float32()
        altitude_msg.data = float(obs[12])
        self.altitude_pub.publish(altitude_msg)

        vertical_velocity_msg = Float32()
        vertical_velocity_msg.data = float(obs[5])
        self.vertical_velocity_pub.publish(vertical_velocity_msg)

        if done:
            self.get_logger().info("Episode bitti. Viewer kapanmadan roket resetleniyor.")
            self.obs, self.info = self.env.reset_with_viewer()


def main(args=None):
    rclpy.init(args=args)

    node = MujocoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
