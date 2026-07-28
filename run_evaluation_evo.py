# run_evo_comparison.py

from pathlib import Path
from evo.core import metrics, sync
from evo.core.units import Unit
from evo.tools import file_interface


def main():
    gt_path = Path("results/gt/corridor4/trajectory_ground_truth.txt")
    # est_path = Path("results/openvins/corridor4/trajectory_openvins.txt")
    est_path = Path("results/orbslam3/corridor4/trajectory_orbslam3.txt")

    print("--- 1. Ingesting Data via evo ---")
    traj_gt = file_interface.read_tum_trajectory_file(gt_path)
    traj_est = file_interface.read_tum_trajectory_file(est_path)
    print(f"Loaded Ground Truth:       {traj_gt.num_poses} poses")
    print(f"Loaded OpenVINS Estimate:  {traj_est.num_poses} poses")

    print("\n--- 2. Synchronizing and SE(3) Aligning via evo ---")
    # Associate / synchronize matching timestamps
    traj_gt_sync, traj_est_sync = sync.associate_trajectories(
        traj_gt, traj_est, max_diff=0.01
    )
    print(f"Synchronized matching frames: {traj_gt_sync.num_poses}")

    # Umeyama SE(3) alignment (in-place)
    traj_est_sync.align(traj_gt_sync, correct_scale=False)
    print("Spatial alignment completed successfully.")

    data = (traj_gt_sync, traj_est_sync)

    print("\n--- 3. Computing evo Reference Metrics ---")

    # ATE (Translation RMSE)
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data(data)
    ate_rmse = ape_metric.get_statistic(metrics.StatisticsType.rmse)

    # RPE delta=1 frame (Translation & Rotation)
    rpe_1_trans = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=1,
        delta_unit=Unit.frames,
        all_pairs=False,
    )
    rpe_1_trans.process_data(data)

    rpe_1_rot = metrics.RPE(
        metrics.PoseRelation.rotation_angle_deg,
        delta=1,
        delta_unit=Unit.frames,
        all_pairs=False,
    )
    rpe_1_rot.process_data(data)

    # RPE delta=30 frames (Translation & Rotation)
    rpe_30_trans = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=30,
        delta_unit=Unit.frames,
        all_pairs=False,
    )
    rpe_30_trans.process_data(data)

    rpe_30_rot = metrics.RPE(
        metrics.PoseRelation.rotation_angle_deg,
        delta=30,
        delta_unit=Unit.frames,
        all_pairs=False,
    )
    rpe_30_rot.process_data(data)

    # Drift over 10m segments (Translation & Rotation)
    rpe_10m_trans = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=10,
        delta_unit=Unit.meters,
        all_pairs=False,
    )
    rpe_10m_trans.process_data(data)

    rpe_10m_rot = metrics.RPE(
        metrics.PoseRelation.rotation_angle_deg,
        delta=10,
        delta_unit=Unit.meters,
        all_pairs=False,
    )
    rpe_10m_rot.process_data(data)

    # Display results formatted identically to run_evaluation.py
    print("\n==========================================================")
    print("                 EVO REFERENCE RESULTS")
    print("==========================================================")
    print(" 1. GLOBAL ACCURACY")
    print(
        f"    - Absolute Trajectory Error (ATE RMSE):  {ate_rmse * 100:.2f} cm"
    )

    print("\n 2. RELATIVE POSE ERROR (RPE)")
    print("    - Frame-to-Frame (delta=1):")
    print(
        f"        * Translation RMSE:                 {rpe_1_trans.get_statistic(metrics.StatisticsType.rmse) * 1000:.2f} mm"
    )
    print(
        f"        * Rotation RMSE:                    {rpe_1_rot.get_statistic(metrics.StatisticsType.rmse):.3f}°"
    )
    print("    - 1-Second Step (delta=30):")
    print(
        f"        * Translation RMSE:                 {rpe_30_trans.get_statistic(metrics.StatisticsType.rmse) * 100:.2f} cm"
    )
    print(
        f"        * Rotation RMSE:                    {rpe_30_rot.get_statistic(metrics.StatisticsType.rmse):.3f}°"
    )

    print("\n 3. DISTANCE-NORMALIZED DRIFT (10m Windows)")
    print(
        f"    - Total Path Length:                    {traj_gt_sync.path_length:.2f} m"
    )
    # evo output in meters over 10m segment -> divide by 10 and multiply by 100 to get %
    drift_trans_mean_pct = (
        rpe_10m_trans.get_statistic(metrics.StatisticsType.mean) / 10.0
    ) * 100.0
    drift_trans_rmse_pct = (
        rpe_10m_trans.get_statistic(metrics.StatisticsType.rmse) / 10.0
    ) * 100.0
    drift_rot_mean_deg_m = (
        rpe_10m_rot.get_statistic(metrics.StatisticsType.mean) / 10.0
    )
    drift_rot_rmse_deg_m = (
        rpe_10m_rot.get_statistic(metrics.StatisticsType.rmse) / 10.0
    )

    print(
        f"    - Translational Drift (Mean):           {drift_trans_mean_pct:.2f} %"
    )
    print(
        f"    - Translational Drift (RMSE):           {drift_trans_rmse_pct:.2f} %"
    )
    print(
        f"    - Rotational Drift (Mean):              {drift_rot_mean_deg_m:.3f} °/m"
    )
    print(
        f"    - Rotational Drift (RMSE):              {drift_rot_rmse_deg_m:.3f} °/m"
    )
    print("==========================================================\n")


if __name__ == "__main__":
    main()