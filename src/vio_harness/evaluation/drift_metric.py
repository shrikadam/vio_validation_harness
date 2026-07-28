# src/vio_harness/evaluation/drift_metric.py

import numpy as np
from scipy.spatial.transform import Rotation as R
from vio_harness.evaluation.base_metric import MetricStrategy
from vio_harness.ingestion.base_parser import TrajectoryData


class DriftStrategy(MetricStrategy):
    """
    Computes distance-normalized translational (%) and rotational (deg/m) drift
    over sliding spatial windows of specified path length.
    """

    def __init__(self, segment_length_m: float = 10.0):
        self.segment_length_m = segment_length_m

    def compute(self, ground_truth: TrajectoryData, estimate: TrajectoryData) -> dict[str, float]:
        if len(ground_truth.positions) != len(estimate.positions):
            raise ValueError("Trajectories must be synchronized before computing Drift.")

        # 1. Compute cumulative path distance on ground truth
        step_distances = np.linalg.norm(np.diff(ground_truth.positions, axis=0), axis=1)
        cum_distance = np.insert(np.cumsum(step_distances), 0, 0.0)
        total_distance = cum_distance[-1]

        if total_distance <= 0:
            raise ValueError("Trajectory has zero total length.")

        # 2. Evaluate sliding segments matching segment_length_m
        trans_drifts_pct = []
        rot_drifts_deg_m = []

        r_gt = R.from_quat(ground_truth.orientations)
        r_est = R.from_quat(estimate.orientations)

        for i in range(len(cum_distance)):
            target_dist = cum_distance[i] + self.segment_length_m
            j = np.searchsorted(cum_distance, target_dist)
            if j >= len(cum_distance):
                break

            actual_dist = cum_distance[j] - cum_distance[i]
            if actual_dist <= 0:
                continue

            # Local translation error over distance L
            gt_sub = r_gt[i].inv().apply(ground_truth.positions[j] - ground_truth.positions[i])
            est_sub = r_est[i].inv().apply(estimate.positions[j] - estimate.positions[i])
            trans_err = np.linalg.norm(gt_sub - est_sub)
            trans_drifts_pct.append((trans_err / actual_dist) * 100.0)

            # Local rotation error over distance L
            r_gt_sub = r_gt[i].inv() * r_gt[j]
            r_est_sub = r_est[i].inv() * r_est[j]
            rot_err_deg = np.degrees((r_gt_sub.inv() * r_est_sub).magnitude())
            rot_drifts_deg_m.append(rot_err_deg / actual_dist)

        # Fallback if trajectory is shorter than segment_length_m
        if not trans_drifts_pct:
            ate_final = np.linalg.norm(ground_truth.positions[-1] - estimate.positions[-1])
            trans_drifts_pct.append((ate_final / total_distance) * 100.0)
            rot_drifts_deg_m.append(0.0)

        return {
            "total_distance_m": float(total_distance),
            "trans_drift_pct_mean": float(np.mean(trans_drifts_pct)),
            "trans_drift_pct_rmse": float(np.sqrt(np.mean(np.array(trans_drifts_pct) ** 2))),
            "rot_drift_deg_m_mean": float(np.mean(rot_drifts_deg_m)),
            "rot_drift_deg_m_rmse": float(np.sqrt(np.mean(np.array(rot_drifts_deg_m) ** 2))),
        }