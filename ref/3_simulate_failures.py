import numpy as np
from scipy.spatial.transform import Rotation as R

def load_tum(filename):
    # Load dataset, ignoring comment lines starting with '#'
    return np.loadtxt(filename, comments='#')

def save_tum(filename, data):
    np.savetxt(filename, data, fmt='%.6f')

print("Loading Ground Truth...")
gt_data = load_tum("gt.txt")

# 1. LATENCY ERROR (Time-sync failure)
# We simulate a perception system that is 15ms late.
# We do this by artificially shifting the timestamps forward by 0.015s.
print("Generating Latency Error...")
est_lat = gt_data.copy()
est_lat[:, 0] += 0.015 
save_tum("est_latency.txt", est_lat)

# 2. SCALE ERROR (Monocular SLAM failure)
# We simulate a 5% scale drift in the estimated world size.
# We multiply only the X, Y, Z translations by 1.05. Quaternions remain untouched.
print("Generating Scale Error...")
est_scale = gt_data.copy()
est_scale[:, 1:4] *= 1.05
save_tum("est_scale.txt", est_scale)

# 3. GRAVITY BLEED (IMU orientation failure)
# We simulate a 1-degree pitch error in the IMU-to-Camera calibration.
# A tilted IMU incorrectly projects the 9.81m/s^2 gravity vector into the horizontal plane.
print("Generating Gravity Bleed (Quadratic Drift)...")
est_grav = gt_data.copy()

# Apply a 1-degree pitch rotation to all quaternions
error_rot = R.from_euler('y', 1.0, degrees=True)
quats = R.from_quat(est_grav[:, 4:8]) # TUM format is qx, qy, qz, qw
corrupted_quats = (error_rot * quats).as_quat()
est_grav[:, 4:8] = corrupted_quats

# Simulate the double-integration translation error over time
# error = 0.5 * (gravity * sin(theta)) * t^2
t0 = est_grav[0, 0]
dt = est_grav[:, 0] - t0
accel_error = 9.81 * np.sin(np.radians(1.0)) # ~0.17 m/s^2 of parasitic acceleration
est_grav[:, 1] += 0.5 * accel_error * (dt ** 2) # Inject quadratic drift into the X-axis

save_tum("est_gravity.txt", est_grav)
print("Done! Output files generated.")