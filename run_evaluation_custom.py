# run_evaluation.py

from pathlib import Path
from vio_harness.evaluation.alignment import TrajectoryProcessor
from vio_harness.evaluation.ate_metric import ATEStrategy
from vio_harness.evaluation.drift_metric import DriftStrategy
from vio_harness.evaluation.jitter_metric import JitterStrategy
from vio_harness.evaluation.rpe_metric import RPEStrategy
from vio_harness.ingestion.tum_parser import TUMParser


def main():
    # 1. Define paths to your real data
    gt_path = Path("results/gt/corridor4/trajectory_ground_truth.txt")
    # est_path = Path("results/openvins/corridor4/trajectory_openvins.txt")
    est_path = Path("results/orbslam3/corridor4/trajectory_orbslam3.txt")

    print("--- 1. Ingesting Real Data ---")
    gt_parser = TUMParser()
    ov_parser = TUMParser()

    gt_data = gt_parser.parse(gt_path)
    est_data = ov_parser.parse(est_path)
    print(f"Loaded Ground Truth:       {len(gt_data.timestamps)} poses")
    print(f"Loaded OpenVINS Estimate:  {len(est_data.timestamps)} poses")

    print("\n--- 2. Pre-processing (Temporal Sync & SE(3) Alignment) ---")
    # Synchronize timestamps (interpolating GT to match OpenVINS frame timestamps)
    sync_gt, sync_est = TrajectoryProcessor.synchronize_trajectories(
        gt_data, est_data
    )
    print(f"Synchronized matching frames: {len(sync_gt.timestamps)}")

    # Align OpenVINS coordinate frame to the motion capture room origin via Umeyama SVD
    aligned_est = TrajectoryProcessor.align_umeyama(sync_gt, sync_est)
    print("Spatial alignment completed successfully.")

    print("\n--- 3. Computing Evaluation Metrics ---")
    # Instantiate strategy objects
    ate_strat = ATEStrategy()
    rpe_step1_strat = RPEStrategy(
        delta_step=1
    )  # Local frame-to-frame relative pose error
    rpe_step30_strat = RPEStrategy(
        delta_step=30
    )  # 1-second step relative pose error (@30Hz)
    drift_strat = DriftStrategy(
        segment_length_m=10.0
    )  # Distance-normalized drift over 10m segments
    jitter_strat = JitterStrategy(
        cutoff_hz=5.0, sample_rate_hz=30.0
    )  # High-pass jitter & jerk

    # Calculate metrics
    ate_rmse = ate_strat.compute(sync_gt, aligned_est)
    rpe_step1 = rpe_step1_strat.compute(sync_gt, aligned_est)
    rpe_step30 = rpe_step30_strat.compute(sync_gt, aligned_est)
    drift_res = drift_strat.compute(sync_gt, aligned_est)
    jitter_res = jitter_strat.compute(sync_gt, aligned_est)

    # Display results
    print("\n==========================================================")
    print("             VIO_HARNESS VALIDATION RESULTS")
    print("==========================================================")
    print(" 1. GLOBAL ACCURACY")
    print(
        f"    - Absolute Trajectory Error (ATE RMSE):  {ate_rmse * 100:.2f} cm"
    )

    print("\n 2. RELATIVE POSE ERROR (RPE)")
    print("    - Frame-to-Frame (delta=1):")
    print(
        f"        * Translation RMSE:                 {rpe_step1['trans_rmse_m'] * 1000:.2f} mm"
    )
    print(
        f"        * Rotation RMSE:                    {rpe_step1['rot_rmse_deg']:.3f}°"
    )
    print("    - 1-Second Step (delta=30):")
    print(
        f"        * Translation RMSE:                 {rpe_step30['trans_rmse_m'] * 100:.2f} cm"
    )
    print(
        f"        * Rotation RMSE:                    {rpe_step30['rot_rmse_deg']:.3f}°"
    )

    print("\n 3. DISTANCE-NORMALIZED DRIFT (10m Windows)")
    print(
        f"    - Total Path Length:                    {drift_res['total_distance_m']:.2f} m"
    )
    print(
        f"    - Translational Drift (Mean):           {drift_res['trans_drift_pct_mean']:.2f} %"
    )
    print(
        f"    - Translational Drift (RMSE):           {drift_res['trans_drift_pct_rmse']:.2f} %"
    )
    print(
        f"    - Rotational Drift (Mean):              {drift_res['rot_drift_deg_m_mean']:.3f} °/m"
    )
    print(
        f"    - Rotational Drift (RMSE):              {drift_res['rot_drift_deg_m_rmse']:.3f} °/m"
    )

    print("\n 4. SPATIAL NOISE & STABILITY (XR Perception)")
    print(
        f"    - High-Pass Jitter RMS (>5 Hz):         {jitter_res['jitter_highpass_rms_mm']:.2f} mm"
    )
    print(
        f"    - Error Jerk RMS:                       {jitter_res['jerk_error_rms_m_s3']:.2f} m/s³"
    )
    print("==========================================================\n")


if __name__ == "__main__":
    main()