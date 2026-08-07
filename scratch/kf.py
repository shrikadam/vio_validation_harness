import numpy as np

class KalmanFilter1D:
    def __init__(self, dt: float, process_noise_std: float, meas_noise_std: float):
        """
        State Vector x = [position, velocity]^T
        """
        self.dt = dt
        
        # Initial State Guess [position=0, velocity=0]
        self.x = np.array([[0.0], 
                           [0.0]])
        
        # Initial Covariance Matrix (high initial uncertainty)
        self.P = np.array([[1000.0,    0.0],
                           [   0.0, 1000.0]])
        
        # State Transition Matrix F (p_k = p_{k-1} + v_{k-1} * dt)
        self.F = np.array([[1.0, dt],
                           [0.0, 1.0]])
        
        # Measurement Matrix H (we only directly measure position)
        self.H = np.array([[1.0, 0.0]])
        
        # Process Noise Covariance Matrix Q
        # Derived from continuous white noise acceleration model
        q_var = process_noise_std ** 2
        self.Q = np.array([[(dt**4)/4, (dt**3)/2],
                           [(dt**3)/2,     dt**2]]) * q_var
        
        # Measurement Noise Covariance Matrix R
        self.R = np.array([[meas_noise_std ** 2]])

    def predict(self) -> np.ndarray:
        """Step 1: Predict the next state using the motion model."""
        # Project state ahead: x_k = F * x_{k-1}
        self.x = self.F @ self.x
        
        # Project uncertainty ahead: P_k = F * P_{k-1} * F^T + Q
        self.P = (self.F @ self.P @ self.F.T) + self.Q
        return self.x

    def update(self, z: float) -> np.ndarray:
        """Step 2: Correct the prediction using incoming sensor measurement z."""
        z_vec = np.array([[z]])
        
        # Measurement residual (Innovation): y = z - H * x
        y = z_vec - (self.H @ self.x)
        
        # Innovation covariance: S = H * P * H^T + R
        S = (self.H @ self.P @ self.H.T) + self.R
        
        # Optimal Kalman Gain: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update State Estimate: x = x + K * y
        self.x = self.x + (K @ y)
        
        # Update State Covariance: P = (I - K * H) * P
        I = np.eye(self.P.shape[0])
        self.P = (I - (K @ self.H)) @ self.P
        
        return self.x

# Example usage
if __name__ == "__main__":
    dt = 0.1
    kf = KalmanFilter1D(dt=dt, process_noise_std=0.2, meas_noise_std=1.5)
    
    # Ground truth trajectory with noisy measurements
    true_velocity = 5.0  # m/s
    noisy_measurements = [i * true_velocity * dt + np.random.normal(0, 1.5) for i in range(10)]
    
    print(f"{'Step':<5} | {'Noisy Measurement':<18} | {'KF Estimated Pos':<16} | {'KF Estimated Vel':<16}")
    print("-" * 65)
    for step, z in enumerate(noisy_measurements):
        kf.predict()
        est_state = kf.update(z)
        pos, vel = est_state[0, 0], est_state[1, 0]
        print(f"{step:<5} | {z:<18.2f} | {pos:<16.2f} | {vel:<16.2f}")