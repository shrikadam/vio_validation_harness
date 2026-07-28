# src/vio_harness/evaluation/ate_metric.py

import numpy as np
from vio_harness.evaluation.base_metric import MetricStrategy
from vio_harness.ingestion.base_parser import TrajectoryData

class ATEStrategy(MetricStrategy):
    """
    Computes the Root Mean Square Error (RMSE) of the Absolute Trajectory Error.
    Requires trajectories to be temporally synchronized and spatially aligned first.
    """
    
    def compute(self, ground_truth: TrajectoryData, estimate: TrajectoryData) -> float:
        # Validate that the arrays match in length (should be handled by sync engine)
        if len(ground_truth.positions) != len(estimate.positions):
            raise ValueError("Trajectories must be synchronized before computing ATE.")
            
        # Compute Euclidean distance between corresponding points
        # axis=1 performs row-wise L2 norm calculation for the 3D vectors
        position_errors = np.linalg.norm(ground_truth.positions - estimate.positions, axis=1)
        
        # Calculate Root Mean Square Error (RMSE)
        rmse_ate = np.sqrt(np.mean(position_errors ** 2))
        
        return float(rmse_ate)