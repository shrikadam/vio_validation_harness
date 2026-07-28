# src/vio_harness/evaluation/jitter_metric.py

import numpy as np
from scipy.signal import butter, filtfilt
from vio_harness.evaluation.base_metric import MetricStrategy
from vio_harness.ingestion.base_parser import TrajectoryData


class JitterStrategy(MetricStrategy):
    """
    Quantifies high-frequency spatial noise (jitter) using high-pass filtering
    and discrete residual jerk analysis.
    """

    def __init__(self, cutoff_hz: float = 5.0, sample_rate_hz: float = 30.0):
        self.cutoff_hz = cutoff_hz
        self.sample_rate_hz = sample_rate_hz

    def compute(self, ground_truth: TrajectoryData, estimate: TrajectoryData) -> dict[str, float]:
        if len(ground_truth.positions) != len(estimate.positions):
            raise ValueError("Trajectories must be synchronized before computing Jitter.")

        # 1. Compute position residuals: r(t) = p_est(t) - p_gt(t)
        residuals = estimate.positions - ground_truth.positions  # shape (N, 3)

        # 2. High-Pass Filter Residuals to isolate >5Hz jitter
        nyquist = 0.5 * self.sample_rate_hz
        normal_cutoff = self.cutoff_hz / nyquist

        if normal_cutoff < 1.0 and len(residuals) > 15:
            b, a = butter(2, normal_cutoff, btype="high", analog=False)
            jitter_signal = filtfilt(b, a, residuals, axis=0)
            jitter_rms_m = np.sqrt(np.mean(jitter_signal**2))
        else:
            jitter_rms_m = 0.0

        # 3. Compute Discrete Error Jerk (3rd derivative of spatial residuals)
        dt = np.median(np.diff(ground_truth.timestamps))
        if dt <= 0:
            dt = 1.0 / self.sample_rate_hz

        # d^3(residuals) / dt^3
        accel_err = np.diff(residuals, n=2, axis=0) / (dt**2)
        jerk_err = np.diff(accel_err, n=1, axis=0) / dt
        jerk_rms = np.sqrt(np.mean(jerk_err**2)) if len(jerk_err) > 0 else 0.0

        return {
            "jitter_highpass_rms_mm": float(jitter_rms_m * 1000.0),  # mm
            "jerk_error_rms_m_s3": float(jerk_rms),  # m/s^3
        }