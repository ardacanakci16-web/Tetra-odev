import numpy as np
from collections import deque

from rocket_mujoco_ros.config import (
    IMU_ANGULAR_VELOCITY_NOISE_STD,
    IMU_UP_VECTOR_NOISE_STD,
    IMU_DELAY_STEPS,
)


class ImuSensor:
    def __init__(self):
        self.buffer = deque(maxlen=IMU_DELAY_STEPS + 1)

    def read(self, observation):
        observation = np.asarray(observation, dtype=np.float32)

        up_vector = observation[6:9]
        angular_velocity = observation[9:12]

        up_vector_noise = np.random.normal(0.0, IMU_UP_VECTOR_NOISE_STD, size=3)
        angular_velocity_noise = np.random.normal(
            0.0,
            IMU_ANGULAR_VELOCITY_NOISE_STD,
            size=3
        )

        data = {
            "up_vector": (up_vector + up_vector_noise).astype(np.float32),
            "angular_velocity": (angular_velocity + angular_velocity_noise).astype(np.float32),
        }

        self.buffer.append(data)

        if len(self.buffer) < self.buffer.maxlen:
            return data

        return self.buffer[0]