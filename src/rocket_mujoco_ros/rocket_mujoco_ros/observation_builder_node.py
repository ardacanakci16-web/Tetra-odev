import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point, Vector3

from rocket_mujoco_ros.config import (
    TOPIC_RL_OBSERVATION,
    TOPIC_COMMAND,
    COMMAND_SIZE,
    CMD_LAND,
)


class ObservationBuilderNode(Node):
    def __init__(self):
        super().__init__("observation_builder_node")

        self.position = None
        self.velocity = None
        self.altitude = None
        self.up_vector = None
        self.angular_velocity = None
        self.command_one_hot = np.zeros(COMMAND_SIZE, dtype=np.float32)
        self.command_one_hot[CMD_LAND] = 1.0

        self.create_subscription(
            Int32,
            TOPIC_COMMAND,
            self.command_callback,
            10
        )


        self.obs_pub = self.create_publisher(
            Float32MultiArray,
            TOPIC_RL_OBSERVATION,
            10
        )

        self.create_subscription(
            Point,
            "/rocket/gps/position",
            self.position_callback,
            10
        )

        self.create_subscription(
            Vector3,
            "/rocket/gps/velocity",
            self.velocity_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/rocket/barometer/altitude",
            self.altitude_callback,
            10
        )

        self.create_subscription(
            Vector3,
            "/rocket/imu/up_vector",
            self.up_vector_callback,
            10
        )

        self.create_subscription(
            Vector3,
            "/rocket/imu/angular_velocity",
            self.angular_velocity_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.publish_observation)

        self.get_logger().info("Observation builder node started.")

    def position_callback(self, msg):
        self.position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def velocity_callback(self, msg):
        self.velocity = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def altitude_callback(self, msg):
        self.altitude = float(msg.data)

    def up_vector_callback(self, msg):
        self.up_vector = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def angular_velocity_callback(self, msg):
        self.angular_velocity = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def command_callback(self, msg):
        command_id = int(msg.data)

        self.command_one_hot = np.zeros(COMMAND_SIZE, dtype=np.float32)

        if 0 <= command_id < COMMAND_SIZE:
            self.command_one_hot[command_id] = 1.0


    def publish_observation(self):
        if (
            self.position is None or
            self.velocity is None or
            self.altitude is None or
            self.up_vector is None or
            self.angular_velocity is None
        ):
            return

        observation = np.concatenate([
            self.position,          # 0-2
            self.velocity,          # 3-5
            self.up_vector,         # 6-8
            self.angular_velocity,  # 9-11
            np.array([self.altitude], dtype=np.float32),
            self.command_one_hot,  # 12
        ]).astype(np.float32)

        msg = Float32MultiArray()
        msg.data = observation.astype(float).tolist()
        self.obs_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ObservationBuilderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
