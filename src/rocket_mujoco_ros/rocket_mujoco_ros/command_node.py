import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

from rocket_mujoco_ros.config import (
    TOPIC_COMMAND,
    CMD_TAKEOFF,
    CMD_HOVER,
    CMD_DESCEND,
    CMD_LAND,
)


class CommandNode(Node):
    def __init__(self):
        super().__init__("command_node")

        self.command_pub = self.create_publisher(
            Int32,
            TOPIC_COMMAND,
            10
        )

        self.current_command = CMD_LAND

        self.timer = self.create_timer(0.1, self.publish_command)

        self.get_logger().info(
            "T=TAKEOFF H=HOVER D=DESCEND L=LAND ESC=quit"
        )

    def get_key(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def publish_command(self):
        key = self.get_key()

        if key == "\x1b":
            rclpy.shutdown()
            return

        if key is None:
            pass
        elif key.lower() == "t":
            self.current_command = CMD_TAKEOFF
            self.get_logger().info(">>> KOMUT: TAKEOFF (Kalkış) <<<")
        elif key.lower() == "h":
            self.current_command = CMD_HOVER
            self.get_logger().info(">>> KOMUT: HOVER (Havada Asılı Kal) <<<")
        elif key.lower() == "d":
            self.current_command = CMD_DESCEND
            self.get_logger().info(">>> KOMUT: DESCEND (Alçal) <<<")
        elif key.lower() == "l":
            self.current_command = CMD_LAND
            self.get_logger().info(">>> KOMUT: LAND (İniş) <<<")

        msg = Int32()
        msg.data = int(self.current_command)

        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    node = CommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
