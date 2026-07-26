# src/vio_harness/fault_injection/imu_tamper.py

import numpy as np
import pandas as pd
from vio_harness.fault_injection.base_injector import BaseTamper

class TimeSyncTamper(BaseTamper):
    """
    Injects a temporal delay into the sensor timestamps, simulating 
    driver latency or clock desynchronization between sensors.
    """
    
    def __init__(self, delay_seconds: float):
        self.delay_seconds = delay_seconds

    def apply(self, sensor_data: pd.DataFrame) -> pd.DataFrame:
        tampered_data = sensor_data.copy()
        
        # Shift all timestamps into the future (or past)
        tampered_data['timestamp'] += self.delay_seconds
        
        return tampered_data


class GyroRandomWalkTamper(BaseTamper):
    """
    Injects a drifting bias (Brownian motion / Random Walk) into the gyroscope data.
    This simulates thermal gradients or electrical instability over time.
    """
    
    def __init__(self, noise_density: float):
        """
        Args:
            noise_density (float): The standard deviation of the step size for the random walk 
                                   (e.g., 1e-4 rad/s/sqrt(Hz)).
        """
        self.noise_density = noise_density

    def apply(self, sensor_data: pd.DataFrame) -> pd.DataFrame:
        tampered_data = sensor_data.copy()
        
        n_samples = len(tampered_data)
        
        # 1. Generate Gaussian white noise for the steps
        # Shape: (N, 3) for the wx, wy, wz axes
        steps = np.random.normal(loc=0.0, scale=self.noise_density, size=(n_samples, 3))
        
        # 2. Integrate the white noise to create a Random Walk (Brownian motion)
        random_walk_bias = np.cumsum(steps, axis=0)
        
        # 3. Add the drifting bias to the raw gyroscope readings
        tampered_data['wx'] += random_walk_bias[:, 0]
        tampered_data['wy'] += random_walk_bias[:, 1]
        tampered_data['wz'] += random_walk_bias[:, 2]
        
        return tampered_data