# tests/test_faults.py

import pytest
import numpy as np
import pandas as pd
from vio_harness.fault_injection.imu_tamper import TimeSyncTamper, GyroRandomWalkTamper

@pytest.fixture
def clean_imu_data():
    return pd.DataFrame({
        "timestamp": [1.0, 1.01, 1.02],
        "wx": [0.0, 0.0, 0.0],
        "wy": [0.0, 0.0, 0.0],
        "wz": [0.0, 0.0, 0.0],
    })

def test_time_sync_tamper(clean_imu_data):
    """Ensures timestamps are delayed accurately without altering sensor readings."""
    tamper = TimeSyncTamper(delay_seconds=0.015)
    faulty_data = tamper.apply(clean_imu_data)
    
    # Assert time shifted
    assert faulty_data['timestamp'].iloc[0] == pytest.approx(1.015)
    # Assert gyro data untouched
    assert faulty_data['wx'].iloc[0] == 0.0

def test_gyro_random_walk_tamper(clean_imu_data):
    """Ensures the random walk injects non-zero noise."""
    tamper = GyroRandomWalkTamper(noise_density=0.1)
    faulty_data = tamper.apply(clean_imu_data)
    
    # Since it's random, we just assert the array was modified and is not all zeros
    assert not np.allclose(faulty_data['wx'].to_numpy(), clean_imu_data['wx'].to_numpy())