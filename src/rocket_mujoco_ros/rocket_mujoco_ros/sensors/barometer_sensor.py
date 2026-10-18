import numpy as np
from collections import deque

from rocket_mujoco_ros.config import (
    BAROMETER_ALTITUDE_NOISE_STD,
    BAROMETER_ALTITUDE_BIAS,
    BAROMETER_DELAY_STEPS,
)


class BarometerSensor:
    def __init__(self):
        self.buffer = deque(maxlen=BAROMETER_DELAY_STEPS + 1)

    def read(self, observation):
        observation = np.asarray(observation, dtype=np.float32)

        altitude = float(observation[12])
        noise = np.random.normal(0.0, BAROMETER_ALTITUDE_NOISE_STD)

        data = {
            "altitude": float(altitude + BAROMETER_ALTITUDE_BIAS + noise),
        }

        self.buffer.append(data)

        if len(self.buffer) < self.buffer.maxlen:
            return data

        return self.buffer[0]