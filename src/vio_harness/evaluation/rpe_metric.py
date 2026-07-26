# src/vio_harness/evaluation/rpe_metric.py

import numpy as np
from vio_harness.evaluation.base_metric import MetricStrategy
from vio_harness.models.trajectory import TrajectoryData

class TranslationalRPEStrategy(MetricStrategy):
    """
    Computes the Translational Relative Pose Error over a fixed step size.
    Measures the local drift or jitter of the state estimator.
    """
    
    def __init__(self, delta_step: int = 1):
        """
        Args:
            delta_step (int): The frame offset to compute relative motion. 
                              e.g., delta=1 measures frame-to-frame jitter.
                              delta=30 measures drift over 1 second (if tracking at 30Hz).
        """
        self.delta_step = delta_step

    def compute(self, ground_truth: TrajectoryData, estimate: TrajectoryData) -> float:
        if len(ground_truth.positions) != len(estimate.positions):
            raise ValueError("Trajectories must be synchronized before computing RPE.")
            
        if len(ground_truth.positions) <= self.delta_step:
            raise ValueError(f"Trajectory too short for delta step of {self.delta_step}.")

        # 1. Compute the relative translation vector for the Ground Truth
        # Motion from frame i to frame i+delta
        gt_motion = ground_truth.positions[self.delta_step:] - ground_truth.positions[:-self.delta_step]
        
        # 2. Compute the relative translation vector for the Estimate
        est_motion = estimate.positions[self.delta_step:] - estimate.positions[:-self.delta_step]
        
        # 3. Compute the Euclidean distance between the relative motions
        motion_errors = np.linalg.norm(gt_motion - est_motion, axis=1)
        
        # Calculate Root Mean Square Error (RMSE)
        rmse_rpe = np.sqrt(np.mean(motion_errors ** 2))
        
        return float(rmse_rpe)