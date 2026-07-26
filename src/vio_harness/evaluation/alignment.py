# src/vio_harness/evaluation/alignment.py

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d
from vio_harness.models.trajectory import TrajectoryData

class TrajectoryProcessor:
    """Handles temporal synchronization and spatial alignment of trajectories."""

    @staticmethod
    def synchronize_trajectories(gt: TrajectoryData, est: TrajectoryData) -> tuple[TrajectoryData, TrajectoryData]:
        """
        Interpolates the Ground Truth (GT) to match the timestamps of the Estimate (EST).
        Uses Linear Interpolation for translation and SLERP for quaternions.
        """
        # Find the overlapping time window
        start_time = max(gt.timestamps[0], est.timestamps[0])
        end_time = min(gt.timestamps[-1], est.timestamps[-1])
        
        # Mask the estimated trajectory to the valid overlapping window
        valid_est_mask = (est.timestamps >= start_time) & (est.timestamps <= end_time)
        sync_timestamps = est.timestamps[valid_est_mask]
        sync_est_pos = est.positions[valid_est_mask]
        sync_est_quat = est.orientations[valid_est_mask]
        
        # 1. Interpolate Positions (Linear)
        pos_interp_func = interp1d(gt.timestamps, gt.positions, axis=0, kind='linear')
        sync_gt_pos = pos_interp_func(sync_timestamps)
        
        # 2. Interpolate Orientations (SLERP)
        rotations = R.from_quat(gt.orientations)
        slerp = Slerp(gt.timestamps, rotations)
        sync_gt_quat = slerp(sync_timestamps).as_quat()
        
        sync_gt = TrajectoryData(sync_timestamps, sync_gt_pos, sync_gt_quat)
        sync_est = TrajectoryData(sync_timestamps, sync_est_pos, sync_est_quat)
        
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