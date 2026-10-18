import numpy as np
from collections import deque

from rocket_mujoco_ros.config import (
    GPS_POSITION_NOISE_STD,
    GPS_VELOCITY_NOISE_STD,
    GPS_DELAY_STEPS,
)


class GpsSensor:
    def __init__(self):
        self.buffer = deque(maxlen=GPS_DELAY_STEPS + 1)

    def read(self, observation):
        observation = np.asarray(observation, dtype=np.float32)

        position = observation[0:3]
        linear_velocity = observation[3:6]

        position_noise = np.random.normal(0.0, GPS_POSITION_NOISE_STD, size=3)
        velocity_noise = np.random.normal(0.0, GPS_VELOCITY_NOISE_STD, size=3)

        data = {
            "position": (position + position_noise).astype(np.float32),
            "linear_velocity": (linear_velocity + velocity_noise).astype(np.float32),
        }

        self.buffer.append(data)

        if len(self.buffer) < self.buffer.maxlen:
            return data

        return self.buffer[0]