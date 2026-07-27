"""
Here is a robust, object-oriented NumPy implementation that generates a synthetic trajectory, calculates the exact mathematical errors described above, and visualizes the drift. This script avoids heavy third-party geometry libraries so you can see the raw math in action.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PoseValidator:
    """
    A foundational validation toolkit for 3D trajectory analysis.
    Demonstrates pure NumPy implementations of SE(3) and SO(3) error metrics.
    """
    
    @staticmethod
    def normalize_quaternion(q: np.ndarray) -> np.ndarray:
        """Ensures the quaternion is a unit quaternion."""
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else q

    @staticmethod
    def compute_translation_error(t_gt: np.ndarray, t_est: np.ndarray) -> float:
        """Computes Euclidean distance between two 3D position vectors."""
        return np.linalg.norm(t_gt - t_est)

    @staticmethod
    def compute_rotational_error_quaternion(q_gt: np.ndarray, q_est: np.ndarray) -> float:
        """
        Computes the geodesic angular error between two quaternions in radians.
        Expects quaternions in format [w, x, y, z].
        """
        q_gt = PoseValidator.normalize_quaternion(q_gt)
        q_est = PoseValidator.normalize_quaternion(q_est)
        
        # Dot product of quaternions
        dot_product = np.clip(np.dot(q_gt, q_est), -1.0, 1.0)
        
        # Absolute value handles the quaternion double-cover (q and -q are same rotation)
        angular_error = 2 * np.arccos(np.abs(dot_product))
        return angular_error

    @staticmethod
    def generate_synthetic_data(num_frames: int = 100):
        """Generates a ground truth and a noisy estimated trajectory."""
        time_steps = np.linspace(0, 10, num_frames)
        
        # Ground Truth: Moving in a circle while looking forward
        t_gt = np.column_stack([
            np.cos(time_steps) * 2, 
            np.sin(time_steps) * 2, 
            time_steps * 0.1
        ])
        
        # Quaternions [w, x, y, z]: smooth rotation around Z-axis
        q_gt = np.column_stack([
            np.cos(time_steps / 2), 
            np.zeros(num_frames), 
            np.zeros(num_frames), 
            np.sin(time_steps / 2)
        ])
        
        # Estimated: Add accumulating drift (simulating VIO drift)
        drift_t = np.column_stack([
            time_steps * 0.05, 
            time_steps * -0.02, 
            time_steps * 0.01
        ])
        t_est = t_gt + drift_t
        
        # Add slight rotational jitter (simulating sensor noise)
        jitter = np.random.normal(0, 0.02, (num_frames, 4))
        q_est = q_gt + jitter
        
        # Normalize all estimated quaternions
        q_est = np.apply_along_axis(PoseValidator.normalize_quaternion, 1, q_est)
        
        return time_steps, t_gt, q_gt, t_est, q_est

def run_validation_demonstrator():
    print("Initializing Trajectory Validation Pipeline...")
    
    # 1. Load Data
    time_steps, t_gt, q_gt, t_est, q_est = PoseValidator.generate_synthetic_data(150)
    num_frames = len(time_steps)
    
    # 2. Compute Metrics
    trans_errors = []
    rot_errors_deg = []
    
    for i in range(num_frames):
        # Translation Error
        t_err = PoseValidator.compute_translation_error(t_gt[i], t_est[i])
        trans_errors.append(t_err)
        
        # Rotational Error (converted to degrees for readability)
        r_err_rad = PoseValidator.compute_rotational_error_quaternion(q_gt[i], q_est[i])
        rot_errors_deg.append(np.degrees(r_err_rad))
        
    rmse_trans = np.sqrt(np.mean(np.array(trans_errors)**2))
    rmse_rot = np.sqrt(np.mean(np.array(rot_errors_deg)**2))
    
    print(f"Validation Complete.")
    print(f"RMSE Translation Error: {rmse_trans:.4f} meters")
    print(f"RMSE Rotational Error:  {rmse_rot:.4f} degrees")

    # 3. Visualization Dashboard
    fig = plt.figure(figsize=(14, 5))
    
    # Plot 1: 3D Trajectory Map
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(t_gt[:, 0], t_gt[:, 1], t_gt[:, 2], label='Ground Truth (OptiTrack)', color='blue')
    ax1.plot(t_est[:, 0], t_est[:, 1], t_est[:, 2], label='Estimated (VIO)', color='red', linestyle='--')
    ax1.set_title("3D Pose Trajectory Drift")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")
    ax1.legend()

    # Plot 2: Error over Time
    ax2 = fig.add_subplot(122)
    ax2.plot(time_steps, trans_errors, label='Translation Error (m)', color='purple')
    ax2.plot(time_steps, rot_errors_deg, label='Rotational Error (deg)', color='orange')
    ax2.set_title("Pose Error Over Time (ATE)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error Magnitude")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_validation_demonstrator()