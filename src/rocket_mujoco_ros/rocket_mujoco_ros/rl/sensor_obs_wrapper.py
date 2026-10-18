import gymnasium as gym
import numpy as np
from collections import deque
from gymnasium import spaces

from rocket_mujoco_ros.config import (
    IMU_ANGULAR_VELOCITY_NOISE_STD,
    IMU_UP_VECTOR_NOISE_STD,
    IMU_DELAY_STEPS,
    BAROMETER_ALTITUDE_NOISE_STD,
    BAROMETER_ALTITUDE_BIAS,
    BAROMETER_DELAY_STEPS,
    GPS_POSITION_NOISE_STD,
    GPS_VELOCITY_NOISE_STD,
    GPS_DELAY_STEPS,
)

class SensorObservationWrapper(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        
        self.imu_buffer = deque(maxlen=IMU_DELAY_STEPS + 1)
        self.baro_buffer = deque(maxlen=BAROMETER_DELAY_STEPS + 1)
        self.gps_buffer = deque(maxlen=GPS_DELAY_STEPS + 1)

    def reset(self, **kwargs):
        self.imu_buffer.clear()
        self.baro_buffer.clear()
        self.gps_buffer.clear()
        
        obs, info = self.env.reset(**kwargs)
        
        # Başlangıçta rastgele bir komut seç (Ajanın her durumu öğrenmesi için)
        self.env.unwrapped.set_command(np.random.randint(0, 4))
        
        # Fill buffers with initial state
        for _ in range(max(IMU_DELAY_STEPS, BAROMETER_DELAY_STEPS, GPS_DELAY_STEPS) + 1):
            noisy_obs = self.observation(obs)
            
        return noisy_obs, info

    def step(self, action):
        # Eğitim sırasında her 400 adımda (8 saniyede) bir komutu rastgele değiştir
        if self.env.unwrapped.step_count % 400 == 0:
            self.env.unwrapped.set_command(np.random.randint(0, 4))
            
        obs, reward, terminated, truncated, info = self.env.step(action)
        return self.observation(obs), reward, terminated, truncated, info

    def observation(self, obs):
        obs = np.asarray(obs, dtype=np.float32)

        # True states
        pos = obs[0:3]
        lin_vel = obs[3:6]
        up_vector = obs[6:9]
        ang_vel = obs[9:12]
        altitude = obs[12]
        command = obs[13:17]

        # GPS Noise
        pos_noise = np.random.normal(0.0, GPS_POSITION_NOISE_STD, size=3)
        vel_noise = np.random.normal(0.0, GPS_VELOCITY_NOISE_STD, size=3)
        self.gps_buffer.append((pos + pos_noise, lin_vel + vel_noise))
        noisy_pos, noisy_vel = self.gps_buffer[0]

        # IMU Noise
        up_noise = np.random.normal(0.0, IMU_UP_VECTOR_NOISE_STD, size=3)
        ang_noise = np.random.normal(0.0, IMU_ANGULAR_VELOCITY_NOISE_STD, size=3)
        self.imu_buffer.append((up_vector + up_noise, ang_vel + ang_noise))
        noisy_up, noisy_ang = self.imu_buffer[0]

        # Barometer Noise
        alt_noise = np.random.normal(0.0, BAROMETER_ALTITUDE_NOISE_STD)
        self.baro_buffer.append(altitude + BAROMETER_ALTITUDE_BIAS + alt_noise)
        noisy_alt = self.baro_buffer[0]

        noisy_obs = np.concatenate([
            noisy_pos,
            noisy_vel,
            noisy_up,
            noisy_ang,
            np.array([noisy_alt], dtype=np.float32),
            command
        ]).astype(np.float32)

        return noisy_obs
