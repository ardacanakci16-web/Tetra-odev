import os

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Int32

from rocket_mujoco_ros.config import (
    TOPIC_RL_OBSERVATION,
    TOPIC_RAW_ACTION,
    TOPIC_COMMAND,
    ACTION_SIZE,
    SIM_DT,
    CMD_TARGET_ALTITUDE,
    CMD_LAND,
    # Yükseklik PID
    ALT_POS_KP,
    ALT_VEL_KP,
    ALT_VEL_KI,
    ALT_VEL_KD,
    ALT_VEL_I_MAX,
    HOVER_THRUST,
    TARGET_VEL_MAX_UP,
    TARGET_VEL_MAX_DOWN,
    # Yanal PID
    LAT_POS_KP,
    LAT_VEL_KP,
    LAT_MAX_TILT,
    # Attitude PID
    ATT_KP,
    ATT_KD,
    # Trajectory
    TRAJECTORY_SMOOTH_RATE,
)

try:
    from stable_baselines3 import PPO
except ImportError:
    PPO = None


class RlAgentNode(Node):
    def __init__(self):
        super().__init__("rl_agent_node")

        self.observation = None

        # --- RL Model Yükleme ---
        model_path = os.path.expanduser("~/rocket_logs/models/rocket_model_final.zip")
        self.model = None
        if PPO is not None and os.path.exists(model_path):
            try:
                self.model = PPO.load(model_path)
                self.get_logger().info(f"Yapay zeka modeli yüklendi: {model_path}")
            except Exception as e:
                self.get_logger().error(f"Model yüklenirken hata oluştu: {e}")
        else:
            self.get_logger().info(
                "Eğitilmiş model bulunamadı. Uzman PID kontrolcü kullanılacak."
            )

        # --- PID Durum Değişkenleri ---
        self.vel_integral = 0.0       # Dikey hız integral terimi
        self.prev_vel_error = 0.0     # Dikey hız önceki hata (türev için)
        self.smoothed_target_alt = 11.0  # Yumuşatılmış hedef irtifa (başlangıç: havada)

        # --- ROS2 Arayüzleri ---
        self.action_pub = self.create_publisher(
            Float32MultiArray,
            TOPIC_RAW_ACTION,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            TOPIC_RL_OBSERVATION,
            self.observation_callback,
            10
        )

        self.timer = self.create_timer(SIM_DT, self.control_loop)

        from rocket_mujoco_ros.config import CMD_HOVER
        self.direct_command = CMD_HOVER
        self.create_subscription(Int32, TOPIC_COMMAND, self.direct_cmd_callback, 10)

        self.get_logger().info("RL Agent node started (Cascaded PID Expert).")

    def direct_cmd_callback(self, msg):
        self.direct_command = int(msg.data)

    def observation_callback(self, msg):
        self.observation = np.array(msg.data, dtype=np.float32)

    def control_loop(self):
        if self.observation is None:
            return

        # Eğitilmiş model varsa ve yeterli adımda eğitildiyse onu kullan:
        # if self.model is not None:
        #     action, _states = self.model.predict(self.observation, deterministic=True)
        # else:
        action = self.expert_policy(self.observation)

        msg = Float32MultiArray()
        msg.data = action.astype(float).tolist()

        self.action_pub.publish(msg)

    def expert_policy(self, obs):
        """
        Cascaded PID Uzman Kontrolcü

        Observation vektörü:
          0-2   position       (x, y, z)
          3-5   velocity       (vx, vy, vz)
          6-8   up_vector      (ux, uy, uz)
          9-11  angular_vel    (wx, wy, wz)
          12    altitude       (z tekrar)
          13-16 command_one_hot (takeoff, hover, descend, land)
        """
        # --------------------------------------------------
        # Observation'dan durum çıkartma
        # --------------------------------------------------
        pos_x, pos_y = float(obs[0]), float(obs[1])
        vel_x, vel_y, vel_z = float(obs[3]), float(obs[4]), float(obs[5])
        up_x, up_y, up_z = float(obs[6]), float(obs[7]), float(obs[8])
        ang_vel_x, ang_vel_y = float(obs[9]), float(obs[10])
        altitude = float(obs[12])

        target_alt = CMD_TARGET_ALTITUDE.get(self.direct_command, CMD_TARGET_ALTITUDE[CMD_LAND])

        # ==================================================
        # AŞAMA 1: Yörünge Yumuşatıcı (Trajectory Smoother)
        # ==================================================
        # Hedef irtifayı aniden değiştirmek yerine, saniyede
        # TRAJECTORY_SMOOTH_RATE metre hızla kademeli yaklaş
        alt_diff = target_alt - self.smoothed_target_alt
        max_change = TRAJECTORY_SMOOTH_RATE * SIM_DT
        if abs(alt_diff) > max_change:
            self.smoothed_target_alt += np.sign(alt_diff) * max_change
        else:
            self.smoothed_target_alt = target_alt

        # ==================================================
        # AŞAMA 2: Yükseklik PID (Cascaded: Konum → Hız)
        # ==================================================
        # Dış döngü: konum hatası → hedef dikey hız
        alt_error = self.smoothed_target_alt - altitude
        target_vel_z = ALT_POS_KP * alt_error
        target_vel_z = np.clip(target_vel_z, TARGET_VEL_MAX_DOWN, TARGET_VEL_MAX_UP)

        # İç döngü: hız hatası → itki komutu (PID)
        vel_error = target_vel_z - vel_z

        # P terimi
        p_term = ALT_VEL_KP * vel_error

        # I terimi (anti-windup ile)
        self.vel_integral += vel_error * SIM_DT
        self.vel_integral = np.clip(self.vel_integral, -ALT_VEL_I_MAX, ALT_VEL_I_MAX)
        i_term = ALT_VEL_KI * self.vel_integral

        # D terimi
        vel_error_derivative = (vel_error - self.prev_vel_error) / SIM_DT
        d_term = ALT_VEL_KD * vel_error_derivative
        self.prev_vel_error = vel_error

        thrust = HOVER_THRUST + p_term + i_term + d_term
        thrust = np.clip(thrust, 0.0, 1.0)

        # ==================================================
        # AŞAMA 3: Yanal Konum PID (X,Y sürüklenme düzeltme)
        # ==================================================
        # Hedef konum: pad merkezi (0, 0)
        # Dış döngü: konum hatası → hedef yanal hız
        target_vel_x = -LAT_POS_KP * pos_x
        target_vel_y = -LAT_POS_KP * pos_y

        # İç döngü: hız hatası → hedef eğim (tilt)
        # X ekseni etrafında dönme (tilt_x) Y ekseninde kuvvet üretir.
        # Y ekseni etrafında dönme (tilt_y) X ekseninde kuvvet üretir.
        desired_tilt_y = -LAT_VEL_KP * (target_vel_x - vel_x)
        desired_tilt_x = LAT_VEL_KP * (target_vel_y - vel_y)

        desired_tilt_x = np.clip(desired_tilt_x, -LAT_MAX_TILT, LAT_MAX_TILT)
        desired_tilt_y = np.clip(desired_tilt_y, -LAT_MAX_TILT, LAT_MAX_TILT)

        # ==================================================
        # AŞAMA 4: Attitude PID (Dik Durma + Yanal Tilt Hedefi)
        # ==================================================
        # up_vector sapması: dik durma hedefi (0,0,1) + yanal tilt
        # up_y → X ekseni etrafında dönme (torque_x)
        # up_x → Y ekseni etrafında dönme (torque_y), ters işaret
        att_error_x = (up_y - desired_tilt_x)
        att_error_y = -(up_x + desired_tilt_y)

        torque_x = ATT_KP * att_error_x - ATT_KD * ang_vel_x
        torque_y = ATT_KP * att_error_y - ATT_KD * ang_vel_y

        # ==================================================
        # Çıkış
        # ==================================================
        action = np.array([thrust, torque_x, torque_y], dtype=np.float32)

        self.get_logger().info(
            f"Komut: {self.direct_command}, Hedef: {target_alt:.2f}, Irtifa: {altitude:.2f}, Itki: {thrust:.3f}",
            throttle_duration_sec=1.0
        )

        return np.clip(action, -1.0, 1.0)


def main(args=None):
    rclpy.init(args=args)

    node = RlAgentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()