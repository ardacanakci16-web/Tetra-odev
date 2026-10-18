import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3, Point

from rocket_mujoco_ros.config import TOPIC_OBSERVATION
from rocket_mujoco_ros.sensors.imu_sensor import ImuSensor
from rocket_mujoco_ros.sensors.barometer_sensor import BarometerSensor
from rocket_mujoco_ros.sensors.gps_sensor import GpsSensor


class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")

        self.imu_sensor = ImuSensor()
        self.barometer_sensor = BarometerSensor()
        self.gps_sensor = GpsSensor()

        self.observation_sub = self.create_subscription(
            Float32MultiArray,
            TOPIC_OBSERVATION,
            self.observation_callback,
            10
        )

        self.imu_up_pub = self.create_publisher(
            Vector3,
            "/rocket/imu/up_vector",
            10
        )

        self.imu_angular_velocity_pub = self.create_publisher(
            Vector3,
            "/rocket/imu/angular_velocity",
            10
        )

        self.altitude_pub = self.create_publisher(
            Float32,
            "/rocket/barometer/altitude",
            10
        )

        self.gps_position_pub = self.create_publisher(
            Point,
            "/rocket/gps/position",
            10
        )

        self.gps_velocity_pub = self.create_publisher(
            Vector3,
            "/rocket/gps/velocity",
            10
        )

        self.get_logger().info("Sensor node started.")

    def observation_callback(self, msg):
        observation = msg.data

        imu_data = self.imu_sensor.read(observation)
        barometer_data = self.barometer_sensor.read(observation)
        gps_data = self.gps_sensor.read(observation)

        up_msg = Vector3()
        up_msg.x = float(imu_data["up_vector"][0])
        up_msg.y = float(imu_data["up_vector"][1])
        up_msg.z = float(imu_data["up_vector"][2])
        self.imu_up_pub.publish(up_msg)

        angular_msg = Vector3()
        angular_msg.x = float(imu_data["angular_velocity"][0])
        angular_msg.y = float(imu_data["angular_velocity"][1])
        angular_msg.z = float(imu_data["angular_velocity"][2])
        self.imu_angular_velocity_pub.publish(angular_msg)

        altitude_msg = Float32()
        altitude_msg.data = float(barometer_data["altitude"])
        self.altitude_pub.publish(altitude_msg)

        position_msg = Point()
        position_msg.x = float(gps_data["position"][0])
        position_msg.y = float(gps_data["position"][1])
        position_msg.z = float(gps_data["position"][2])
        self.gps_position_pub.publish(position_msg)

        velocity_msg = Vector3()
        velocity_msg.x = float(gps_data["linear_velocity"][0])
        velocity_msg.y = float(gps_data["linear_velocity"][1])
        velocity_msg.z = float(gps_data["linear_velocity"][2])
        self.gps_velocity_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
