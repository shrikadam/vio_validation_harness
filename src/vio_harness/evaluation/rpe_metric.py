# src/vio_harness/evaluation/rpe_metric.py

import numpy as np
from scipy.spatial.transform import Rotation as R
from vio_harness.evaluation.base_metric import MetricStrategy
from vio_harness.ingestion.base_parser import TrajectoryData

class RPEStrategy(MetricStrategy):
    """
    Computes Full SE(3) Relative Pose Error (RPE) for both translation and rotation
    over a fixed step size.
    """

    def __init__(self, delta_step: int = 1):
        """
        Args:
            delta_step (int): Frame offset to compute relative motion.
                              e.g., delta=1 measures frame-to-frame jitter.
                              delta=30 measures drift over 1 second (at 30Hz).
        """
        self.delta_step = delta_step

    def compute(self, ground_truth: TrajectoryData, estimate: TrajectoryData) -> dict[str, float]:
        if len(ground_truth.positions) != len(estimate.positions):
            raise ValueError("Trajectories must be synchronized before computing RPE.")

        if len(ground_truth.positions) <= self.delta_step:
            raise ValueError(f"Trajectory too short for delta step of {self.delta_step}.")

        # 1. Slice poses into start (i) and end (j) frames for the step delta
        p_gt_i = ground_truth.positions[: -self.delta_step]
        p_gt_j = ground_truth.positions[self.delta_step :]
        p_est_i = estimate.positions[: -self.delta_step]
        p_est_j = estimate.positions[self.delta_step :]

        r_gt_i = R.from_quat(ground_truth.orientations[: -self.delta_step])
        r_gt_j = R.from_quat(ground_truth.orientations[self.delta_step :])
        r_est_i = R.from_quat(estimate.orientations[: -self.delta_step])
        r_est_j = R.from_quat(estimate.orientations[self.delta_step :])

        # 2. Compute relative motions in local body frame
        # Relative Rotation: R_rel = R_i^-1 * R_j
        r_gt_rel = r_gt_i.inv() * r_gt_j
        r_est_rel = r_est_i.inv() * r_est_j

        # Relative Translation: p_rel = R_i^-1 * (p_j - p_i)
        p_gt_rel = r_gt_i.inv().apply(p_gt_j - p_gt_i)
        p_est_rel = r_est_i.inv().apply(p_est_j - p_est_i)

        # 3. Compute Translation Errors
        trans_errors = np.linalg.norm(p_gt_rel - p_est_rel, axis=1)

        # 4. Compute Rotation Errors on SO(3)
        # Error Rotation: R_err = R_gt_rel^-1 * R_est_rel
        r_err = r_gt_rel.inv() * r_est_rel
        rot_errors_deg = np.degrees(r_err.magnitude())

        return {
            "trans_rmse_m": float(np.sqrt(np.mean(trans_errors**2))),
            "trans_mean_m": float(np.mean(trans_errors)),
            "rot_rmse_deg": float(np.sqrt(np.mean(rot_errors_deg**2))),
            "rot_mean_deg": float(np.mean(rot_errors_deg)),
        }