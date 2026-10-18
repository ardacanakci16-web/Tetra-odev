import os
import time

import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from rocket_mujoco_ros.config import SIM_DT

CMD_TAKEOFF = 0
CMD_HOVER = 1
CMD_DESCEND = 2
CMD_LAND = 3

COMMAND_SIZE = 4


class FreeJointRocketTerrainLandingEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode=None):
        super().__init__()

        self.render_mode = render_mode
        self.viewer = None

        if get_package_share_directory:
            package_share_dir = get_package_share_directory('rocket_mujoco_ros')
            xml_path = os.path.join(package_share_dir, "assets", "hopper_default.xml")
        else:
            base_dir = os.path.dirname(__file__)
            xml_path = os.path.join(base_dir, "assets", "hopper_default.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.hopper_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "hopper"
        )

        self.thrust_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "thrust_site"
        )

        self.max_thrust = 18.0
        self.max_tvc_deg = 12.0

        self.step_count = 0
        self.max_steps = 3000

        self.current_command = CMD_LAND

        self.command_one_hot = np.zeros(
            COMMAND_SIZE,
            dtype=np.float32
        )

        self.command_one_hot[CMD_LAND] = 1.0

        self.target_altitude = 1.5

        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(17,),
            dtype=np.float32
        )

    def set_command(self, command_id):
        command_id = int(command_id)

        if command_id < 0 or command_id >= COMMAND_SIZE:
            command_id = CMD_HOVER

        self.current_command = command_id

        self.command_one_hot = np.zeros(
            COMMAND_SIZE,
            dtype=np.float32
        )

        self.command_one_hot[command_id] = 1.0

        if command_id == CMD_TAKEOFF:
            self.target_altitude = 2.5

        elif command_id == CMD_HOVER:
            self.target_altitude = 1.5

        elif command_id == CMD_DESCEND:
            self.target_altitude = 0.8

        elif command_id == CMD_LAND:
            self.target_altitude = 0.35

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.step_count = 0

        mujoco.mj_resetData(self.model, self.data)

        self.data.qpos[0:3] = np.array(
            [0.0, 0.0, 0.355]
        )

        self.data.qpos[3:7] = np.array(
            [1.0, 0.0, 0.0, 0.0]
        )

        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0

        self.set_command(CMD_LAND)

        mujoco.mj_forward(self.model, self.data)

        return self.get_obs(), {}

    def step(self, action):
        self.step_count += 1

        action = np.asarray(
            action,
            dtype=np.float32
        )

        action = np.clip(
            action,
            self.action_space.low,
            self.action_space.high
        )

        thrust_cmd = float(action[0])
        yaw_cmd = float(action[1])
        pitch_cmd = float(action[2])

        self.data.ctrl[0] = (
            yaw_cmd * self.max_tvc_deg
        )

        self.data.ctrl[1] = (
            pitch_cmd * self.max_tvc_deg
        )

        n_frames = max(1, int(SIM_DT / self.model.opt.timestep))
        for _ in range(n_frames):
            self.apply_thrust(thrust_cmd)
            mujoco.mj_step(self.model, self.data)

        if self.viewer is not None:
            self.viewer.sync()

        obs = self.get_obs()

        reward = self.compute_reward(
            obs,
            action
        )

        altitude = float(obs[12])
        up_z = float(obs[8])

        vertical_velocity = float(obs[5])

        crashed = (
            altitude < 0.18
            and abs(vertical_velocity) > 1.2
        )

        tipped_over = up_z < 0.45

        timeout = (
            self.step_count >= self.max_steps
        )

        terminated = bool(
            crashed or tipped_over
        )

        truncated = bool(timeout)

        info = {
            "altitude": altitude,
            "target_altitude": self.target_altitude,
            "command": int(self.current_command),
            "crashed": bool(crashed),
            "tipped_over": bool(tipped_over),
        }

        return (
            obs,
            float(reward),
            terminated,
            truncated,
            info
        )

    def apply_thrust(self, thrust_cmd):
        thrust = (
            thrust_cmd * self.max_thrust
        )

        site_xmat = self.data.site_xmat[
            self.thrust_site_id
        ].reshape(3, 3)

        thrust_dir = (
            site_xmat
            @ np.array([0.0, 0.0, 1.0])
        )

        force = thrust_dir * thrust

        self.data.xfrc_applied[:] = 0.0

        self.data.xfrc_applied[
            self.hopper_body_id,
            0:3
        ] = force

    def get_obs(self):
        pos = self.data.xpos[
            self.hopper_body_id
        ].copy()

        rot = self.data.xmat[
            self.hopper_body_id
        ].reshape(3, 3).copy()

        lin_vel = self.data.qvel[0:3].copy()

        ang_vel = self.data.qvel[3:6].copy()

        up_vector = (
            rot
            @ np.array([0.0, 0.0, 1.0])
        )

        altitude = np.array(
            [pos[2]],
            dtype=np.float32
        )

        obs = np.concatenate([
            pos.astype(np.float32),
            lin_vel.astype(np.float32),
            up_vector.astype(np.float32),
            ang_vel.astype(np.float32),
            altitude,
            self.command_one_hot,
        ]).astype(np.float32)

        return obs

    def compute_reward(self, obs, action):
        pos_x = float(obs[0])
        pos_y = float(obs[1])
        altitude = float(obs[12])

        vel_x = float(obs[3])
        vel_y = float(obs[4])
        vertical_velocity = float(obs[5])

        up_z = float(obs[8])

        angular_speed = float(
            np.linalg.norm(obs[9:12])
        )

        lateral_dist = np.sqrt(pos_x**2 + pos_y**2)
        lateral_speed = np.sqrt(vel_x**2 + vel_y**2)

        altitude_error = abs(
            altitude
            - self.target_altitude
        )

        reward = 0.0

        # --- Temel Cezalar ---
        reward -= 2.5 * altitude_error         # İrtifa hatasını cezalandır
        reward -= 0.5 * abs(vertical_velocity)  # Dikey hız sapmasını cezalandır
        reward -= 0.15 * angular_speed           # Aşırı dönmeyi cezalandır
        reward -= 1.2 * lateral_dist             # ← YENİ: Yanal sürüklenmeyi cezalandır
        reward -= 0.3 * lateral_speed            # ← YENİ: Yanal hızı cezalandır

        # --- Dik Durma Ödülü ---
        reward += 2.5 * up_z

        # --- Yakıt Tasarrufu ---
        reward -= 0.01 * float(action[0])

        # --- Komuta Özgü Ödüller ---
        if self.current_command == CMD_TAKEOFF:

            if vertical_velocity > 0.0:
                reward += 1.5

            if altitude >= self.target_altitude:
                reward += 8.0

        elif self.current_command == CMD_HOVER:

            if (
                altitude_error < 0.15
                and abs(vertical_velocity) < 0.25
                and lateral_dist < 0.3
            ):
                reward += 6.0

        elif self.current_command == CMD_DESCEND:

            target_descent_speed = -0.35

            reward -= abs(
                vertical_velocity
                - target_descent_speed
            )

        elif self.current_command == CMD_LAND:

            reward -= 1.5 * altitude

            if (
                altitude < 0.40
                and abs(vertical_velocity) < 0.25
                and up_z > 0.85
                and lateral_dist < 0.25
            ):
                reward += 12.0

        return reward

    def launch_viewer(self):
        self.close_viewer()

        self.viewer = (
            mujoco.viewer.launch_passive(
                self.model,
                self.data
            )
        )

        return self.viewer

    def reset_with_viewer(
        self,
        seed=None,
        options=None
    ):
        obs, info = self.reset(
            seed=seed,
            options=options
        )

        if (
            self.viewer is None
            or not self.viewer.is_running()
        ):
            self.launch_viewer()

        return obs, info

    def close_viewer(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def close(self):
        self.close_viewer()