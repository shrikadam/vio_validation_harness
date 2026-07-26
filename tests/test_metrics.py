# tests/test_metrics.py

import pytest
import numpy as np
from vio_harness.models.trajectory import TrajectoryData
from vio_harness.evaluation.ate_metric import ATEStrategy
from vio_harness.evaluation.rpe_metric import TranslationalRPEStrategy

@pytest.fixture
def perfect_trajectory():
    return TrajectoryData(
        timestamps=np.array([0.0, 1.0, 2.0]),
        positions=np.array([[0,0,0], [1,0,0], [2,0,0]]),
        orientations=np.array([[0,0,0,1], [0,0,0,1], [0,0,0,1]])
    )

def test_ate_strategy_with_known_offset(perfect_trajectory):
    """ATE should exactly match a hardcoded physical offset."""
    # Create a trajectory offset by exactly 2.0 meters on the Y axis
    offset_trajectory = TrajectoryData(
        timestamps=perfect_trajectory.timestamps,
        positions=perfect_trajectory.positions + np.array([0, 2.0, 0]),
        orientations=perfect_trajectory.orientations
    )
    
    metric = ATEStrategy()
    error = metric.compute(perfect_trajectory, offset_trajectory)
    
    assert error == pytest.approx(2.0), "ATE computation failed math verification"

def test_rpe_strategy_perfect_match(perfect_trajectory):
    """RPE on identical trajectories should be exactly 0."""
    metric = TranslationalRPEStrategy(delta_step=1)
    error = metric.compute(perfect_trajectory, perfect_trajectory)
    
    assert error == pytest.approx(0.0), "RPE should be zero for identical trajectories"