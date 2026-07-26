# run_evaluation.py

from pathlib import Path
from vio_harness.ingestion.tum_parser import TUMParser
from vio_harness.ingestion.openvins_parser import OpenVINSParser
from vio_harness.evaluation.alignment import TrajectoryProcessor
from vio_harness.evaluation.ate_metric import ATEStrategy
from vio_harness.evaluation.rpe_metric import TranslationalRPEStrategy

import pandas as pd

def main():
    # 1. Define paths to your real data
    gt_path = Path("results/gt/corridor4/trajectory_ground_truth.txt")
    est_path = Path("results/openvins/corridor4/trajectory_openvins.txt")
    est_csv_path = Path("results/openvins/corridor4/trajectory_openvins.csv")

    df = pd.read_csv(est_path, sep=r"\s+", comment="#", header=None)
    df.columns = ["timestamp", "px", "py", "pz", "qx", "qy", "qz", "qw"]
    df.to_csv(est_csv_path, index=False)

    print("--- 1. Ingesting Real Data ---")
    gt_parser = TUMParser()
    ov_parser = OpenVINSParser()
    
    gt_data = gt_parser.parse(gt_path)
    est_data = ov_parser.parse(est_csv_path)
    print(f"Loaded Ground Truth: {len(gt_data.timestamps)} poses")
    print(f"Loaded OpenVINS Estimate: {len(est_data.timestamps)} poses")
    
    print("\n--- 2. Pre-processing (Temporal Sync & SE(3) Alignment) ---")
    # Synchronize timestamps (interpolating GT to match OpenVINS frame rate)
    sync_gt, sync_est = TrajectoryProcessor.synchronize_trajectories(gt_data, est_data)
    print(f"Synchronized matching frames: {len(sync_gt.timestamps)}")
    
    # Align OpenVINS coordinate frame to the TUM-VI motion capture room origin via Umeyama SVD
    aligned_est = TrajectoryProcessor.align_umeyama(sync_gt, sync_est)
    print("Spatial alignment completed successfully.")
    
    print("\n--- 3. Computing Evaluation Metrics ---")
    # Instantiate strategies
    ate_strategy = ATEStrategy()
    rpe_strategy_local = TranslationalRPEStrategy(delta_step=1)      # Frame-to-frame jitter
    rpe_strategy_drift = TranslationalRPEStrategy(delta_step=30)     # Drift over ~1 second (assuming 30Hz)
    
    # Run calculations
    ate_rmse = ate_strategy.compute(sync_gt, aligned_est)
    rpe_local = rpe_strategy_local.compute(sync_gt, aligned_est)
    rpe_drift = rpe_strategy_drift.compute(sync_gt, aligned_est)
    
    print(f"========================================")
    print(f" REAL-WORLD VALIDATION RESULTS (OpenVINS)")
    print(f"========================================")
    print(f" Absolute Trajectory Error (ATE RMSE): {ate_rmse * 100:.2f} cm")
    print(f" Local Jitter (RPE delta=1):           {rpe_local * 1000:.2f} mm")
    print(f" Drift over 1s (RPE delta=30):         {rpe_drift * 100:.2f} cm")
    print(f"========================================")

if __name__ == "__main__":
    main()