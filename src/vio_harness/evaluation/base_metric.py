# src/vio_harness/evaluation/base_metric.py

import abc
from vio_harness.models.trajectory import TrajectoryData

class MetricStrategy(abc.ABC):
    """
    Strategy interface for computing evaluation metrics like ATE or RPE.
    Assumes that the trajectories have already been temporally synchronized 
    and spatially aligned.
    """
    
    @abc.abstractmethod
    def compute(self, ground_truth: TrajectoryData, estimate: TrajectoryData) -> float:
        """
        Computes the metric between ground truth and estimated trajectories.
        
        Args:
            ground_truth (TrajectoryData): The reference trajectory (e.g., TUM-VI GT).
            estimate (TrajectoryData): The VIO output to evaluate.
        
        Returns:
            float: The computed error metric (e.g., RMSE in meters).
        """
        pass