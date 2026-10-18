import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray, Bool, String, Float64MultiArray
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

import numpy as np
import json
from scipy.spatial.transform import Rotation

from rocket_mujoco_ros.config import (
    ACTION_SIZE,
    TOPIC_ACTION,
    TOPIC_OBSERVATION,
    TOPIC_REWARD,
    TOPIC_DONE,
    TOPIC_INFO,
    SIM_DT,
    COMMAND_SIZE,
    CMD_LAND
)


class GazeboNode(Node):
    def __init__(self):
        super().__init__("gazebo_node")

        self.action = np.zeros(ACTION_SIZE, dtype=np.float32)
        
        # State tracking
        self.pos = np.zeros(3)
        self.lin_vel = np.zeros(3)
        self.up_vector = np.array([0.0, 0.0, 1.0])
        self.ang_vel = np.zeros(3)
        self.altitude = 0.355
        
        from rocket_mujoco_ros.config import CMD_HOVER
        self.command_one_hot = np.zeros(COMMAND_SIZE, dtype=np.float32)
        self.command_one_hot[CMD_HOVER] = 1.0

        self.target_altitude = 1.5
        self.current_command = CMD_HOVER
        
        self.max_thrust = 18.0
        self.max_tvc_rad = 12.0 * (np.pi / 180.0)
        self.step_count = 0
        self.max_steps = 3000

        # --- Publishers ---
        self.observation_pub = self.create_publisher(Float32MultiArray, TOPIC_OBSERVATION, 10)
        self.reward_pub = self.create_publisher(Float32, TOPIC_REWARD, 10)
        self.done_pub = self.create_publisher(Bool, TOPIC_DONE, 10)
        self.info_pub = self.create_publisher(String, TOPIC_INFO, 10)
        self.altitude_pub = self.create_publisher(Float32, "/rocket/state/altitude", 10)
        self.vertical_velocity_pub = self.create_publisher(Float32, "/rocket/state/vertical_velocity", 10)
        
        # Gazebo specific publishers
        self.thrust_pub = self.create_publisher(Wrench, "/rocket/thrust", 10)
        self.tvc_pub = self.create_publisher(Float64MultiArray, "/tvc_controller/commands", 10)

        # --- Subscribers ---
        self.action_sub = self.create_subscription(Float32MultiArray, TOPIC_ACTION, self.action_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/rocket/state", self.odom_callback, 10)

        # --- Timer ---
        self.timer = self.create_timer(SIM_DT, self.step_environment)

        self.get_logger().info("Gazebo Rocket ROS2 node started.")

    def action_callback(self, msg):
        if len(msg.data) != ACTION_SIZE:
            return
        self.action = np.array(msg.data, dtype=np.float32)

    def odom_callback(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.up_vector = rot @ np.array([0.0, 0.0, 1.0])
        
        self.lin_vel[0] = msg.twist.twist.linear.x
        self.lin_vel[1] = msg.twist.twist.linear.y
        self.lin_vel[2] = msg.twist.twist.linear.z
        
        self.ang_vel[0] = msg.twist.twist.angular.x
        self.ang_vel[1] = msg.twist.twist.angular.y
        self.ang_vel[2] = msg.twist.twist.angular.z
        
        self.altitude = self.pos[2]

    def get_obs(self):
        obs = np.concatenate([
            self.pos.astype(np.float32),
            self.lin_vel.astype(np.float32),
            self.up_vector.astype(np.float32),
            self.ang_vel.astype(np.float32),
            np.array([self.altitude], dtype=np.float32),
            self.command_one_hot,
        ]).astype(np.float32)
        return obs

    def compute_reward(self, obs, action):
        pos_x, pos_y = float(obs[0]), float(obs[1])
        altitude = float(obs[12])
        vel_x, vel_y = float(obs[3]), float(obs[4])
        vertical_velocity = float(obs[5])
        up_z = float(obs[8])
        angular_speed = float(np.linalg.norm(obs[9:12]))

        lateral_dist = np.sqrt(pos_x**2 + pos_y**2)
        lateral_speed = np.sqrt(vel_x**2 + vel_y**2)
        altitude_error = abs(altitude - self.target_altitude)

        reward = 0.0
        reward -= 2.5 * altitude_error
        reward -= 0.5 * abs(vertical_velocity)
        reward -= 0.15 * angular_speed
        reward -= 1.2 * lateral_dist
        reward -= 0.3 * lateral_speed
        reward += 2.5 * up_z
        reward -= 0.01 * float(action[0])
        return reward

    def step_environment(self):
        self.step_count += 1
        
        # Action clip
        action = np.clip(self.action, -1.0, 1.0)
        thrust_cmd = float(action[0])
        yaw_cmd = float(action[1])
        pitch_cmd = float(action[2])

        # Apply TVC commands
        tvc_msg = Float64MultiArray()
        tvc_msg.data = [yaw_cmd * self.max_tvc_rad, pitch_cmd * self.max_tvc_rad]
        self.tvc_pub.publish(tvc_msg)
        
        # Apply Thrust (MuJoCo UFO physics match - applied at CM)
        theta_x = yaw_cmd * self.max_tvc_rad
        theta_y = pitch_cmd * self.max_tvc_rad
        F = thrust_cmd * self.max_thrust

        wrench_msg = Wrench()
        wrench_msg.force.x = F * float(np.sin(theta_y) * np.cos(theta_x))
        wrench_msg.force.y = F * float(-np.sin(theta_x))
        wrench_msg.force.z = F * float(np.cos(theta_y) * np.cos(theta_x))
        self.thrust_pub.publish(wrench_msg)

        # Get Observation
        obs = self.get_obs()
        reward = self.compute_reward(obs, action)

        crashed = (self.altitude < 0.18 and abs(self.lin_vel[2]) > 1.2)
        tipped_over = self.up_vector[2] < 0.45
        timeout = (self.step_count >= self.max_steps)

        done = crashed or tipped_over or timeout

        # Publish state topics
        obs_msg = Float32MultiArray()
        obs_msg.data = obs.astype(float).tolist()
        self.observation_pub.publish(obs_msg)

        reward_msg = Float32()
        reward_msg.data = float(reward)
        self.reward_pub.publish(reward_msg)

        done_msg = Bool()
        done_msg.data = bool(done)
        self.done_pub.publish(done_msg)

        info = {
            "altitude": float(self.altitude),
            "target_altitude": self.target_altitude,
            "command": int(self.current_command),
            "crashed": bool(crashed),
            "tipped_over": bool(tipped_over),
        }
        info_msg = String()
        info_msg.data = json.dumps(info)
        self.info_pub.publish(info_msg)

        altitude_msg = Float32()
        altitude_msg.data = float(self.altitude)
        self.altitude_pub.publish(altitude_msg)

        vertical_velocity_msg = Float32()
        vertical_velocity_msg.data = float(self.lin_vel[2])
        self.vertical_velocity_pub.publish(vertical_velocity_msg)

        # We cannot reset Gazebo physics easily here without a service call, 
        # so for now we just log it.
        if done:
            self.get_logger().info(
                f"Episode bitti! Nedenler -> crashed: {crashed} (alt={self.altitude:.2f}, vel_z={self.lin_vel[2]:.2f}), "
                f"tipped_over: {tipped_over} (up_z={self.up_vector[2]:.2f}), timeout: {timeout} (step={self.step_count})"
            )
            self.step_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = GazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
