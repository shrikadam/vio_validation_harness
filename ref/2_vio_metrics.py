"""
This script generates an unaligned estimated trajectory, uses Singular Value Decomposition (SVD) to perform the Umeyama alignment, and computes both ATE and RPE.
"""
import numpy as np
import matplotlib.pyplot as plt

class TrajectoryEvaluator:
    """
    Day 2 Toolkit: Alignment, ATE, and RPE for 3D trajectories.
    Assumes time-synchronized trajectories of shape (N, 3) for translation.
    """
    
    @staticmethod
    def align_umeyama(pts_gt: np.ndarray, pts_est: np.ndarray, with_scale: bool = False):
        """
        Aligns pts_est to pts_gt using the Umeyama algorithm via SVD.
        Returns the aligned estimated points and the transformation (R, t, s).
        """
        assert pts_gt.shape == pts_est.shape, "Point clouds must have same shape"
        n = pts_gt.shape[0]

        # 1. Compute centroids
        centroid_gt = np.mean(pts_gt, axis=0)
        centroid_est = np.mean(pts_est, axis=0)

        # 2. Center the points
        gt_centered = pts_gt - centroid_gt
        est_centered = pts_est - centroid_est

        # 3. Compute covariance matrix
        H = est_centered.T @ gt_centered / n

        # 4. Singular Value Decomposition (SVD)
        U, D, Vt = np.linalg.svd(H)
        
        # 5. Calculate Rotation
        R = Vt.T @ U.T
        
        # Handle reflection case (ensure det(R) == 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # 6. Calculate Scale and Translation
        if with_scale:
            var_est = np.var(est_centered, axis=0).sum()
            scale = (1.0 / var_est) * np.sum(D)
        else:
            scale = 1.0

        t = centroid_gt - scale * (R @ centroid_est)

        # 7. Apply transformation
        pts_est_aligned = scale * (pts_est @ R.T) + t

        return pts_est_aligned, R, t, scale

    @staticmethod
    def compute_ate_rmse(pts_gt: np.ndarray, pts_est_aligned: np.ndarray) -> float:
        """Computes Absolute Trajectory Error (RMSE) on aligned translation vectors."""
        errors = np.linalg.norm(pts_gt - pts_est_aligned, axis=1)
        return np.sqrt(np.mean(errors**2))

    @staticmethod
    def compute_rpe_rmse(pts_gt: np.ndarray, pts_est: np.ndarray, delta: int = 1) -> float:
        """
        Computes Relative Pose Error for translation over a fixed interval 'delta'.
        Note: RPE is calculated on the *unaligned* trajectories because it measures relative motion.
        """
        # Calculate motion deltas (step i+delta minus step i)
        motion_gt = pts_gt[delta:] - pts_gt[:-delta]
        motion_est = pts_est[delta:] - pts_est[:-delta]
        
        # Error is the difference between the true motion and estimated motion
        errors = np.linalg.norm(motion_gt - motion_est, axis=1)
        return np.sqrt(np.mean(errors**2))

def run_day2_demonstrator():
    print("--- SLAM Evaluation Pipeline ---")
    
    # Generate synthetic Ground Truth: A simple line in 3D
    N = 100
    pts_gt = np.column_stack([np.linspace(0, 10, N), np.sin(np.linspace(0, 5, N)), np.zeros(N)])
    
    # Generate Estimate: Shifted (wrong origin), rotated, and noisy
    # Simulate an arbitrary starting orientation (e.g. 45 deg yaw)
    theta = np.radians(45)
    R_wrong_start = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])
    
    noise = np.random.normal(0, 0.05, (N, 3))
    pts_est = (pts_gt + noise) @ R_wrong_start.T + np.array([5.0, -2.0, 1.0]) # Add translation offset
    
    # 1. Align Trajectories
    pts_est_aligned, R, t, s = TrajectoryEvaluator.align_umeyama(pts_gt, pts_est, with_scale=False)
    print("Umeyama Alignment Complete.")
    
    # 2. Compute Metrics
    ate = TrajectoryEvaluator.compute_ate_rmse(pts_gt, pts_est_aligned)
    rpe_step1 = TrajectoryEvaluator.compute_rpe_rmse(pts_gt, pts_est, delta=1)
    
    print(f"ATE (Global Drift RMSE):    {ate:.4f} m")
    print(f"RPE (Local Drift per step): {rpe_step1:.4f} m")
    
    # 3. Visualization
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(pts_gt[:,0], pts_gt[:,1], pts_gt[:,2], label='Ground Truth', color='blue', linewidth=2)
    ax.plot(pts_est[:,0], pts_est[:,1], pts_est[:,2], label='Raw Estimate (Unaligned)', color='red', linestyle=':')
    ax.plot(pts_est_aligned[:,0], pts_est_aligned[:,1], pts_est_aligned[:,2], label='Aligned Estimate', color='green', linestyle='--')
    
    ax.set_title("Umeyama Trajectory Alignment")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    run_day2_demonstrator()