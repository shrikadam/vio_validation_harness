# tests/test_regression.py

import pytest
import numpy as np
from pathlib import Path

from vio_harness.ingestion.tum_parser import TUMParser
from vio_harness.evaluation.alignment import TrajectoryProcessor
from vio_harness.evaluation.ate_metric import ATEStrategy
from vio_harness.evaluation.rpe_metric import TranslationalRPEStrategy

# ==========================================
# FIXTURES (Setup Code)
# ==========================================

@pytest.fixture(scope="module")
def gt_data():
    """Loads the Ground Truth trajectory once for the entire test module."""
    parser = TUMParser()
    # In a real CI system, this points to a shared network drive or artifact
    gt_path = Path("dataset/tum_vi/room1_ground_truth.txt")
    
    # Mocking data here for the sake of an executable example
    from vio_harness.models.trajectory import TrajectoryData
    n_frames = 100
    return TrajectoryData(
        timestamps=np.linspace(0, 10, n_frames),
        positions=np.zeros((n_frames, 3)),
        orientations=np.array([[0, 0, 0, 1]] * n_frames)
    )

# ==========================================
# REGRESSION TESTS
# ==========================================

@pytest.mark.parametrize("algorithm, log_file", [
    ("OpenVINS", "dataset/tum_vi/openvins_output.txt"),
    ("Kimera-VIO", "dataset/tum_vi/kimera_output.txt")
])
def test_baseline_tracking_accuracy(gt_data, algorithm, log_file):
    """
    Ensures that under ideal conditions, the algorithms do not exceed 
    XR tracking error thresholds.
    """
    # 1. Ingest (Mocking the ingest for the example)
    parser = TUMParser()
    # est_data = parser.parse(Path(log_file))
    est_data = gt_data  # Mocking perfect data
    
    # 2. Process
    sync_gt, sync_est = TrajectoryProcessor.synchronize_trajectories(gt_data, est_data)
    aligned_est = TrajectoryProcessor.align_umeyama(sync_gt, sync_est)
    
    # 3. Evaluate ATE (Global Drift)
    ate_metric = ATEStrategy()
    ate_error = ate_metric.compute(sync_gt, aligned_est)
    
    # 4. Assert with strict thresholds (5 cm max global drift)
    MAX_ALLOWED_ATE = 0.05  
    assert ate_error < MAX_ALLOWED_ATE, \
        f"{algorithm} failed ATE threshold: {ate_error:.4f}m > {MAX_ALLOWED_ATE}m"

def test_local_jitter_thresholds(gt_data):
    """
    Ensures the frame-to-frame local jitter (RPE) remains imperceptible 
    to prevent motion sickness.
    """
    est_data = gt_data # Mocking data
    
    sync_gt, sync_est = TrajectoryProcessor.synchronize_trajectories(gt_data, est_data)
    aligned_est = TrajectoryProcessor.align_umeyama(sync_gt, sync_est)
    
    # RPE over a 1-frame delta
    rpe_metric = TranslationalRPEStrategy(delta_step=1)
    rpe_error = rpe_metric.compute(sync_gt, aligned_est)
    
    # Strict sub-millimeter jitter requirement
    MAX_ALLOWED_JITTER = 0.001 
    assert rpe_error < MAX_ALLOWED_JITTER, \
        f"Algorithm failed Jitter threshold: {rpe_error:.5f}m > {MAX_ALLOWED_JITTER}m"