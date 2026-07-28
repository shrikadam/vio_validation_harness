# src/vio_harness/evaluation/alignment.py

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d
from vio_harness.ingestion.base_parser import TrajectoryData

class TrajectoryProcessor:
    """Handles temporal synchronization and spatial alignment of trajectories."""

    @staticmethod
    def synchronize_trajectories(
        gt: TrajectoryData, est: TrajectoryData, max_diff_sec: float = 0.01
    ) -> tuple[TrajectoryData, TrajectoryData]:
        """Synchronize trajectories by matching nearest timestamps within max_diff_sec.

        Rejects estimates that fall in ground-truth data gaps.
        """
        gt_ts = np.asarray(gt.timestamps)
        est_ts = np.asarray(est.timestamps)

        # 1. Find nearest ground truth index for each estimate timestamp
        idx = np.searchsorted(gt_ts, est_ts)
        idx = np.clip(idx, 0, len(gt_ts) - 1)

        # Check whether current or previous GT index is closer
        idx_prev = np.clip(idx - 1, 0, len(gt_ts) - 1)
        diff_curr = np.abs(gt_ts[idx] - est_ts)
        diff_prev = np.abs(gt_ts[idx_prev] - est_ts)

        best_gt_idx = np.where(diff_prev < diff_curr, idx_prev, idx)
        best_diff = np.minimum(diff_curr, diff_prev)

        # 2. Filter out matches where timestamp difference exceeds max_diff_sec (e.g., 10 ms)
        valid_mask = best_diff <= max_diff_sec

        valid_est_indices = np.where(valid_mask)[0]
        valid_gt_indices = best_gt_idx[valid_mask]

        # 3. Slice synchronized trajectory objects
        sync_gt = TrajectoryData(
            timestamps=gt_ts[valid_gt_indices],
            positions=gt.positions[valid_gt_indices],
            orientations=gt.orientations[valid_gt_indices],
        )

        sync_est = TrajectoryData(
            timestamps=est_ts[valid_est_indices],
            positions=est.positions[valid_est_indices],
            orientations=est.orientations[valid_est_indices],
        )

        return sync_gt, sync_est

    @staticmethod
    def align_umeyama(gt: TrajectoryData, est: TrajectoryData) -> TrajectoryData:
        """
        Aligns the estimated trajectory to the ground truth using the Umeyama algorithm SE(3).
        Assumes the trajectories are already temporally synchronized.
        """
        n = gt.positions.shape[0]
        
        # 1. Compute centroids
        centroid_gt = np.mean(gt.positions, axis=0)
        centroid_est = np.mean(est.positions, axis=0)
        
        # 2. Center the point clouds
        gt_centered = gt.positions - centroid_gt
        est_centered = est.positions - centroid_est
        
        # 3. Covariance matrix & SVD
        H = est_centered.T @ gt_centered / n
        U, S, Vt = np.linalg.svd(H)
        
        # 4. Rotation matrix
        Rot = Vt.T @ U.T
        if np.linalg.det(Rot) < 0:
            Vt[-1, :] *= -1
            Rot = Vt.T @ U.T
            
        # 5. Translation vector (Scale s=1 for SE(3))
        t = centroid_gt - (Rot @ centroid_est)
        
        # 6. Apply Transformation to positions
        aligned_positions = (est.positions @ Rot.T) + t
        
        # 7. Apply Transformation to orientations
        # Convert translation/rotation to a combined quaternion shift
        R_align = R.from_matrix(Rot)
        est_rotations = R.from_quat(est.orientations)
        aligned_rotations = (R_align * est_rotations).as_quat()
        
        return TrajectoryData(
            timestamps=est.timestamps,
            positions=aligned_positions,
            orientations=aligned_rotations
        )