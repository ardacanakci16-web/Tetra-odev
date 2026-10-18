import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from rocket_mujoco_ros.config import (
    ACTION_SIZE,
    TOPIC_RAW_ACTION,
    TOPIC_ACTION,
    MIN_THRUST_CMD,
    MAX_THRUST_CMD,
    MAX_TORQUE_CMD,
    ACTION_SMOOTHING_ALPHA,
)


class ActionSafetyFilterNode(Node):
    def __init__(self):
        super().__init__("action_safety_filter_node")

        self.last_safe_action = np.zeros(ACTION_SIZE, dtype=np.float32)

        self.raw_action_sub = self.create_subscription(
            Float32MultiArray,
            TOPIC_RAW_ACTION,
            self.raw_action_callback,
            10
        )

        self.safe_action_pub = self.create_publisher(
            Float32MultiArray,
            TOPIC_ACTION,
            10
        )

        self.get_logger().info("Action safety filter node started.")

    def raw_action_callback(self, msg):
        if len(msg.data) != ACTION_SIZE:
            self.get_logger().warn(
                f"Raw action boyutu yanlış. Beklenen: {ACTION_SIZE}, gelen: {len(msg.data)}"
            )
            return

        raw_action = np.array(msg.data, dtype=np.float32)

        if not np.all(np.isfinite(raw_action)):
            self.get_logger().warn("Raw action NaN/Inf içeriyor. Komut reddedildi.")
            return

        safe_action = self.filter_action(raw_action)

        out_msg = Float32MultiArray()
        out_msg.data = safe_action.astype(float).tolist()
        self.safe_action_pub.publish(out_msg)

        self.last_safe_action = safe_action

    def filter_action(self, raw_action):
        thrust = np.clip(raw_action[0], MIN_THRUST_CMD, MAX_THRUST_CMD)
        torque_x = np.clip(raw_action[1], -MAX_TORQUE_CMD, MAX_TORQUE_CMD)
        torque_y = np.clip(raw_action[2], -MAX_TORQUE_CMD, MAX_TORQUE_CMD)

        clipped_action = np.array(
            [thrust, torque_x, torque_y],
            dtype=np.float32
        )

        safe_action = (
            ACTION_SMOOTHING_ALPHA * clipped_action
            + (1.0 - ACTION_SMOOTHING_ALPHA) * self.last_safe_action
        ).astype(np.float32)

        return safe_action


def main(args=None):
    rclpy.init(args=args)

    node = ActionSafetyFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
