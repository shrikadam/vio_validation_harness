# TUM-VI Corridor → OpenVINS & Kimera-VIO: Download, Inference, Visualization, and Unified Output

This runbook covers five stages:

1. Download the exact TUM-VI `corridor` files you need
2. Build & run **OpenVINS** on them
3. Build & run **Kimera-VIO** on them
4. Normalize both outputs (+ ground truth) into **one common trajectory schema**
5. Visualize and validate everything with `evo`, in a layout a downstream test script can consume

Assumes Ubuntu 20.04 + ROS1 Noetic (OpenVINS's most battle-tested combo) and a standalone (non-ROS) build of Kimera-VIO. If you're on 18.04/Melodic or ROS2, the same steps apply with the package names swapped — see the linked docs.

---

## 0. Why two downloads per sequence

- **OpenVINS** is driven by `rosbag play`, so you need the **ROS bag** export.
- **Kimera-VIO**'s standalone binary reads a folder directly (no ROS required), so you need the **EuRoC/DSO-style tar** export (a `mav0/` folder with `cam0/`, `cam1/`, `imu0/`, `mocap0/`).

Both are official exports of the same underlying recording, so estimates from both systems are directly comparable, and the `mocap0` folder in the tar gives you one consistent ground-truth source for both.

---

## 1. Download the TUM-VI corridor files

The TUM-VI benchmark ships `corridor1`–`corridor5` (indoor loop, ground truth only at the start/end since the middle isn't covered by the motion-capture volume). Use the **512×512, calibrated** exports — that's the resolution OpenVINS's shipped `tum_vi` config was tuned against, and it's ~3x smaller than the 1024×1024 versions.

```bash
mkdir -p ~/data/tumvi/{bags,euroc}
cd ~/data/tumvi

# ROS bags (for OpenVINS) — calibrated, 512x512
for i in 1 2 3 4 5; do
  wget -c "http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-corridor${i}_512_16.bag" -P bags/
done

# EuRoC/DSO tars (for Kimera-VIO + ground truth) — calibrated, 512x512
for i in 1 2 3 4 5; do
  wget -c "http://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-corridor${i}_512_16.tar" -P euroc/
  tar -xf "euroc/dataset-corridor${i}_512_16.tar" -C euroc/
done
```

Approximate sizes (bag / tar): corridor1 ≈ 5.9GB / 3.3GB, corridor2 ≈ 6.6GB / 3.7GB, corridor3 ≈ 5.7GB / 3.2GB, corridor4 ≈ 1.9GB / 1.1GB, corridor5 ≈ 5.8GB / 3.2GB. corridor4 is the shortest sequence and the cheapest one to sanity-check your pipeline on first.

If a URL 404s (TUM occasionally moves things between `vision.in.tum.de` and `cdn3.vision.in.tum.de`), grab the current link from the [official download page](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset) — the file naming (`dataset-corridorN_512_16.{bag,tar}`) stays the same.

After extraction you should have, per sequence:
```
euroc/dataset-corridor1_512_16/mav0/
  cam0/{data/*.png, data.csv, sensor.yaml}
  cam1/{data/*.png, data.csv, sensor.yaml}
  imu0/{data.csv, sensor.yaml}
  mocap0/data.csv       # partial ground truth: start + end segments only
```

---

## 2. OpenVINS: build and run

### 2.1 Build (ROS 2)

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions libeigen3-dev libboost-all-dev libceres-dev 
mkdir -p ~/ros2_ws_ov/src && cd ~/ros2_ws_ov/src
git clone https://github.com/rpng/open_vins/
```

Safely handle .h imports in OpenVINS, since ROS 2 Jazzy uses .hpp imports.
```bash
cd ~/ros2_ws_ov

# 1. Fix image_transport.h -> conditional .hpp/.h
find src/open_vins/ -type f \( -name "*.h" -o -name "*.cpp" \) -exec sed -i \
  's|#include <image_transport/image_transport.h>|#if __has_include(<image_transport/image_transport.hpp>)\n#include <image_transport/image_transport.hpp>\n#else\n#include <image_transport/image_transport.h>\n#endif|g' {} +

# 2. Fix tf2_geometry_msgs.h -> conditional .hpp/.h
find src/open_vins/ -type f \( -name "*.h" -o -name "*.cpp" \) -exec sed -i \
  's|#include <tf2_geometry_msgs/tf2_geometry_msgs.h>|#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)\n#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>\n#else\n#include <tf2_geometry_msgs/tf2_geometry_msgs.h>\n#endif|g' {} +

# 3. Fix cv_bridge.h -> conditional .hpp/.h
find src/open_vins/ -type f \( -name "*.h" -o -name "*.cpp" \) -exec sed -i \
  's|#include <cv_bridge/cv_bridge.h>|#if __has_include(<cv_bridge/cv_bridge.hpp>)\n#include <cv_bridge/cv_bridge.hpp>\n#else\n#include <cv_bridge/cv_bridge.h>\n#endif|g' {} +

# Build OpenVins
cd ~/ros2_ws_ov && colcon build --symlink-install
source install/setup.bash
```

OpenVINS already ships a verified `tum_vi` config (`config/tum_vi/estimator_config.yaml` + Kalibr-format IMU/camera chain files), so you don't need to recalibrate anything — just point the launch file at it.

---

### 2.2 Run inference + record output in one shot

First, convert ROS 1 bag into ROS 2 format.
```bash
# Install the conversion tool
pip3 install rosbags

# Convert the downloaded ROS 1 bag to a ROS 2 bag format
rosbags-convert --src bags/dataset-corridor4_512_16.bag --dst bags/dataset-corridor4_512_16_ros2
```

Add a recorder node so the estimate is written straight to a TUM-formatted text file (this is the key step for the "unified output" requirement — see §4). 

*Note: Because the `ov_eval` package's `pose_to_file` utility is not fully ported to ROS 2 yet, we use a custom Python script (`scripts/record_poses_openvins.py`) to record the output without compilation errors.*

```bash
mkdir -p ~/code/vio_validation_harness/results/openvins

# terminal 1 — visualize live (leave this open permanently)
rviz2 -d $(ros2 pkg prefix ov_msckf)/share/ov_msckf/launch/display_ros2.rviz

# terminal 2 — launch the estimator in the background
ros2 launch ov_msckf subscribe.launch.py config:=tum_vi &

# terminal 2 (cont.) — record /ov_msckf/poseimu to file
python3 ~/code/vio_validation_harness/scripts/record_poses_openvins.py \
  ~/code/vio_validation_harness/results/openvins/corridor4.txt &

# terminal 3 — feed the bag (ROS 2 bag directories do not use extensions)
ros2 bag play ~/code/vio_validation_harness/data/tumvi/bags/dataset-corridor4_512_16_ros2

```

A bash script to automate the above can be found at `~/code/vio_validation_harness/run_eval_openvins.sh`

The `record_poses_openvins.py` script automatically writes `time(s) px py pz qx qy qz qw` — this is standard TUM trajectory format, meaning no conversion is needed later for evaluation.

---

## 3. Kimera-VIO: build and run

### 3.1 Build (standalone, no ROS required)

Core dependency chain: GTSAM → OpenCV (≥3.4) → OpenGV (built against GTSAM's Eigen) → DBoW2 → Kimera-RPGO → Kimera-VIO. Condensed sequence (see the [official install doc](https://github.com/MIT-SPARK/Kimera-VIO/blob/master/docs/kimera_vio_install.md) for current version pins and platform-specific fixes):

```bash
mkdir -p kimera_ws && cd kimera_ws

sudo apt-get install -y build-essential cmake unzip pkg-config \
  libjpeg-dev libpng-dev libtiff-dev libvtk9-dev libgtk-3-dev \
  libatlas-base-dev gfortran libtbb-dev libgflags-dev libgoogle-glog-dev

# GTSAM
git clone https://github.com/borglab/gtsam.git && cd gtsam && mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=OFF -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON \
  -DGTSAM_TANGENT_PREINTEGRATION=OFF -DCMAKE_BUILD_TYPE=Release ..
sudo make -j$(nproc) install && cd ../..

# OpenCV
sudo apt-get install libopencv-dev libopencv-contrib-dev

# OpenGV (must reuse GTSAM's Eigen)
git clone https://github.com/laurentkneip/opengv.git && cd opengv && mkdir build && cd build
cmake -DEIGEN_INCLUDE_DIR=$HOME/code/vio_validation_harness/kimera_ws/gtsam/gtsam/3rdparty/Eigen -DEIGEN_INCLUDE_DIRS=$HOME/code/vio_validation_harness/kimera_ws/gtsam/gtsam/3rdparty/Eigen ..
sudo make -j$(nproc) install && cd ../..

# DBoW2
git clone https://github.com/dorian3d/DBoW2.git && cd DBoW2 && mkdir build && cd build
cmake .. && sudo make -j$(nproc) install && cd ../..

# Kimera-RPGO
git clone https://github.com/MIT-SPARK/Kimera-RPGO.git && cd Kimera-RPGO && mkdir build && cd build
# Change params.getDiagonalDamping() = true; to params.setDiagonalDamping(true); in src/GenericSolver.cpp
# Change lmParams.getDiagonalDamping() = params_.lm_diagonal_damping; to lmParams.setDiagonalDamping(params_.lm_diagonal_damping); in src/RobustSolver.cpp
cmake .. && sudo make -j$(nproc) install && cd ../..

# Kimera-VIO itself
git clone https://github.com/MIT-SPARK/Kimera-VIO.git && cd Kimera-VIO && mkdir build && cd build
cmake .. && sudo make -j$(nproc)
```

### 3.2 Create a `TumVi` params folder — required, don't skip

Kimera-VIO ships parameter folders for EuRoC only. TUM-VI's cameras are **fisheye (equidistant distortion)**, not the pinhole-radtan model EuRoC uses, so running the default `Euroc` params against TUM-VI data will silently produce garbage or fail frontend tracking. You need a `params/TumVi/` folder with `LeftCameraParams.yaml`, `RightCameraParams.yaml`, and `ImuParams.yaml`.

Populate it from either (a) `mav0/cam0/sensor.yaml` / `mav0/cam1/sensor.yaml` / `mav0/imu0/sensor.yaml` shipped inside the tar you just extracted, or (b) the pre-verified calibration OpenVINS already ships at `open_vins/config/tum_vi/kalibr_imucam_chain.yaml` and `kalibr_imu_chain.yaml` — same sensor rig, already vetted, so reusing it avoids re-deriving intrinsics yourself.

Key fields to set per camera (Kimera-VIO's supported distortion models are `radtan` and `equidistant`):

```yaml
%YAML:1.0
camera_id: cam0
distortion_model: equidistant
intrinsics: [fx, fy, cx, cy]          # from sensor.yaml intrinsics
distortion_coefficients: [k1, k2, k3, k4]
resolution: [512, 512]
T_BS: !!opencv-matrix                 # camera-to-IMU extrinsic, 4x4
  rows: 4
  cols: 4
  dt: d
  data: [ ... ]
rate_hz: 20
```

And for the IMU, use TUM-VI's published noise values (200 Hz):

```yaml
gyroscope_noise_density: 0.00016
gyroscope_random_walk: 0.000022
accelerometer_noise_density: 0.0028
accelerometer_random_walk: 0.00086
imu_rate: 200
```

Copy `params/Euroc/*.yaml` as your starting templates and only edit the camera/IMU-specific fields above — the frontend/backend tracker tuning parameters can stay at their default Euroc values as a first pass.

### 3.3 Run inference + visualize

```bash
mkdir -p ~/results/kimera
for i in 1 2 3 4 5; do
  cd ~/Kimera-VIO
  ./build/stereoVIOEuroc \
    --dataset_path=$HOME/data/tumvi/euroc/dataset-corridor${i}_512_16 \
    --params_folder_path=./params/TumVi \
    --log_output=true \
    --output_path=$HOME/results/kimera/corridor${i}/ \
    --v=0
done
```

This launches Kimera's own real-time 3D viewer (Pangolin/OpenCV window) showing the live pose graph, feature tracks, and — if you enable meshing — the reconstructed mesh, so you get a visualization for free during the run. With `--log_output=true` each run also writes `output_logs/traj_vio.csv` (renamed here to `~/results/kimera/corridor${i}/traj_vio.csv`) — a CSV with header `#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz`. Note the **quaternion is `qw` first** here, unlike the target TUM schema — handled in §4.

---

## 4. Normalize everything into one schema

Downstream trajectory-evaluation pipelines (and `evo`, the standard tool for this) expect the **TUM trajectory format**: a whitespace-separated text file, one pose per line, sorted by ascending timestamp:

```
# timestamp tx ty tz qx qy qz qw
1520531212.143128 0.0123 -0.0451 0.0091 0.0012 0.0034 -0.0002 0.9999
```

- Timestamp: seconds (float), same epoch as the dataset
- Position: meters, in the world/IMU frame
- Quaternion: `x y z w` order (scalar-last)

OpenVINS's `pose_to_file` output is already in this format — no conversion needed.

### 4.1 Convert ground truth (once per sequence, shared by both systems)

`evo` reads EuRoC ground-truth CSVs natively and can save straight to TUM format:

```bash
pip install evo --upgrade --no-binary evo
mkdir -p ~/results/gt
for i in 1 2 3 4 5; do
  evo_traj euroc ~/data/tumvi/euroc/dataset-corridor${i}_512_16/mav0/mocap0/data.csv \
    --save_as_tum
  mv data.tum ~/results/gt/corridor${i}.txt
done
```

Because `corridor` sequences only have ground truth at the start/end, this file will have two disjoint time ranges — that's expected, and `evo` handles it fine via timestamp association.

### 4.2 Convert Kimera-VIO's `traj_vio.csv` → TUM

```python
# tum_convert_kimera.py
import csv, sys

src, dst = sys.argv[1], sys.argv[2]
with open(src) as f_in, open(dst, "w") as f_out:
    reader = csv.reader(f_in)
    header = next(reader)
    for row in reader:
        ts_ns, x, y, z, qw, qx, qy, qz = row[:8]
        t_s = float(ts_ns) * 1e-9
        f_out.write(f"{t_s:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
```

```bash
for i in 1 2 3 4 5; do
  python3 tum_convert_kimera.py \
    ~/results/kimera/corridor${i}/traj_vio.csv \
    ~/results/kimera/corridor${i}/corridor${i}.txt
done
```

### 4.3 Final layout

```
results/
  gt/
    corridor1.txt  corridor2.txt  ...  corridor5.txt
  openvins/
    corridor1.txt  corridor2.txt  ...  corridor5.txt
  kimera/
    corridor1/corridor1.txt   (+ traj_vio.csv, mesh, logs alongside)
    corridor2/corridor2.txt
    ...
```

All nine trajectory files (5 GT + 5 OpenVINS... adjust per how many sequences you actually ran) now share the identical `timestamp tx ty tz qx qy qz qw` schema — this is what you hand to a downstream validator.

---

## 5. Visualize and validate with `evo`

```bash
# Plot estimate vs ground truth for one sequence
evo_traj tum \
  results/openvins/corridor4.txt \
  results/kimera/corridor4/corridor4.txt \
  --ref results/gt/corridor4.txt \
  -a --plot_mode xyz --save_plot results/corridor4_traj.pdf

# Quantitative validation: Absolute Trajectory Error against ground truth
evo_ape tum results/gt/corridor4.txt results/openvins/corridor4.txt \
  -a --save_results results/openvins_corridor4_ape.zip

evo_ape tum results/gt/corridor4.txt results/kimera/corridor4/corridor4.txt \
  -a --save_results results/kimera_corridor4_ape.zip
```

`-a` triggers Umeyama SE(3) alignment, appropriate here since GT only covers the start/end segments. `evo_ape`'s `--save_results` output is a self-contained zip with the full per-timestamp error series and summary stats (RMSE, mean, median, std) in JSON — a convenient, already-structured artifact if your test pipeline wants machine-readable metrics rather than raw trajectories.

For a batch run across all five sequences and both systems:

```bash
for i in 1 2 3 4 5; do
  for sys in openvins kimera; do
    src=results/${sys}/corridor${i}.txt
    [ "$sys" = kimera ] && src=results/kimera/corridor${i}/corridor${i}.txt
    evo_ape tum results/gt/corridor${i}.txt "$src" -a \
      --save_results results/${sys}_corridor${i}_ape.zip
  done
done
```

---

## Common pitfalls

- **Skipping the TumVi params folder for Kimera-VIO**: running with default `Euroc` params against TUM-VI's fisheye images will not error loudly — it'll just track poorly. Confirm `distortion_model: equidistant` is set.
- **Quaternion order mismatches**: EuRoC/Kimera raw outputs are `qw` first; the TUM/evo target schema is `qw` last. Double check any custom conversion script.
- **Re-initializing OpenVINS between bags**: reuse of a `subscribe.launch` process across multiple `rosbag play` runs without restarting the estimator will corrupt the second trajectory — always relaunch per sequence.
- **corridor sequences have no full ground truth**: don't expect an ATE over the entire trajectory length; `evo` will only score the overlapping (start/end) timestamps, which is expected for this sequence family.
