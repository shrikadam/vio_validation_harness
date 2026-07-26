# TUM-VI Corridor4 → OpenVINS (EKF) vs. Basalt vs. ORB-SLAM3 (Factor Graph): Download, Inference, Visualization, Unified Output

Cleaned-up, single-sequence version of this runbook. Scope for now: **corridor4 only** — the shortest TUM-VI corridor sequence, good for iterating quickly across three systems before ever scaling back out to the full corridor1–5 set.

Three systems, two paradigms:

| | OpenVINS | Basalt | ORB-SLAM3 |
|---|---|---|---|
| Paradigm | EKF (MSCKF) | Sliding-window nonlinear optimization + marginalization | Keyframe bundle adjustment + loop closure / relocalization |
| TUM-VI calibration | ships a verified `tum_vi` config | ships `tumvi_512_ds_calib.json` / `tumvi_512_config.json` | ships `TUM-VI.yaml` + timestamp/IMU association files |
| Install | ROS 2 Jazzy + colcon | one-line installer script | plain CMake build, no ROS needed |
| Output format | TUM, native | TUM, native | TUM, native |
| Dataset input | ROS 2 bag | EuRoC/DSO tar folder | EuRoC/DSO tar folder |

All three write TUM format natively — nothing to convert this time around.

---

## 0. Base directory

Everything below assumes `~/code/vio_validation_harness` as the project root:

```
~/code/vio_validation_harness/
  data/tumvi/{bags,euroc}/
  results/{openvins,basalt,orbslam3,gt}/
  scripts/
```

---

## 1. Download the TUM-VI corridor4 files

```bash
BASE=~/code/vio_validation_harness
mkdir -p "$BASE"/data/tumvi/{bags,euroc}
cd "$BASE"/data/tumvi

# ROS bag (OpenVINS) — calibrated, 512x512
wget -c "http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-corridor4_512_16.bag" -P bags/

# EuRoC/DSO tar (Basalt, ORB-SLAM3, and ground truth) — calibrated, 512x512
wget -c "http://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-corridor4_512_16.tar" -P euroc/
tar -xf euroc/dataset-corridor4_512_16.tar -C euroc/
```

~1.9GB (bag) + ~1.1GB (tar). If a URL 404s, grab the current link from the [official download page](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset) — filenames stay `dataset-corridor4_512_16.{bag,tar}`.

```
data/tumvi/euroc/dataset-corridor4_512_16/mav0/
  cam0/{data/*.png, data.csv, sensor.yaml}
  cam1/{data/*.png, data.csv, sensor.yaml}
  imu0/{data.csv, sensor.yaml}
  mocap0/data.csv       # partial ground truth: start + end segments only
```

---

## 2. OpenVINS: build and run (ROS 2 Jazzy)

### 2.1 Build

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions libeigen3-dev libboost-all-dev libceres-dev
mkdir -p ~/ros2_ws_ov/src && cd ~/ros2_ws_ov/src
git clone https://github.com/rpng/open_vins/
```

OpenVINS's headers predate ROS 2 Jazzy's switch to `.hpp` includes for several packages — patch conditionally rather than hard-switching, so the same tree still builds on older ROS 2 distros too:

```bash
cd ~/ros2_ws_ov

# 1. image_transport.h -> conditional .hpp/.h
find src/open_vins/ -type f \( -name "*.h" -o -name "*.cpp" \) -exec sed -i \
  's|#include <image_transport/image_transport.h>|#if __has_include(<image_transport/image_transport.hpp>)\n#include <image_transport/image_transport.hpp>\n#else\n#include <image_transport/image_transport.h>\n#endif|g' {} +

# 2. tf2_geometry_msgs.h -> conditional .hpp/.h
find src/open_vins/ -type f \( -name "*.h" -o -name "*.cpp" \) -exec sed -i \
  's|#include <tf2_geometry_msgs/tf2_geometry_msgs.h>|#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)\n#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>\n#else\n#include <tf2_geometry_msgs/tf2_geometry_msgs.h>\n#endif|g' {} +

# 3. cv_bridge.h -> conditional .hpp/.h
find src/open_vins/ -type f \( -name "*.h" -o -name "*.cpp" \) -exec sed -i \
  's|#include <cv_bridge/cv_bridge.h>|#if __has_include(<cv_bridge/cv_bridge.hpp>)\n#include <cv_bridge/cv_bridge.hpp>\n#else\n#include <cv_bridge/cv_bridge.h>\n#endif|g' {} +

colcon build --symlink-install
source install/setup.bash
```

OpenVINS already ships a verified `tum_vi` config (`config/tum_vi/estimator_config.yaml` + Kalibr-format IMU/camera chain files) — no recalibration needed.

### 2.2 Run inference + record output

Convert the bag once:

```bash
BASE=~/code/vio_validation_harness
pip3 install rosbags
rosbags-convert --src "$BASE"/data/tumvi/bags/dataset-corridor4_512_16.bag \
                --dst "$BASE"/data/tumvi/bags/dataset-corridor4_512_16_ros2
```

*Note: `ov_eval`'s `pose_to_file` isn't fully ported to ROS 2 yet, so `scripts/record_poses_openvins.py` stands in for it — same TUM-format output, no compilation errors.*

```bash
mkdir -p "$BASE"/results/openvins/corridor4

# terminal 1 — visualize live (leave open)
rviz2 -d $(ros2 pkg prefix ov_msckf)/share/ov_msckf/launch/display_ros2.rviz

# terminal 2 — launch the estimator, then record
ros2 launch ov_msckf subscribe.launch.py config:=tum_vi &
python3 "$BASE"/scripts/record_poses_openvins.py "$BASE"/results/openvins/corridor4/trajectory_openvins.txt &

# terminal 3 — feed the bag (ROS 2 bag dirs have no extension)
ros2 bag play "$BASE"/data/tumvi/bags/dataset-corridor4_512_16_ros2
```

`record_poses_openvins.py` writes `time(s) px py pz qx qy qz qw` — TUM format natively, no conversion needed.

---

## 3. Basalt: install and run

### 3.1 Install

```bash
curl -LsSf https://gitlab.com/VladyslavUsenko/basalt/-/raw/master/scripts/install.sh | sh
```

Places binaries in `~/.local/bin` and calibration/config data in `~/.local/etc/basalt/`. Two things this needs that aren't automatic:

```bash
# 1. put the installer's binaries on PATH
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc

# 2. libbasalt.so isn't found at runtime without this
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/shri/.local/lib/' >> ~/.bashrc

source ~/.bashrc
```

No custom params folder, no calibration conversion — TUM-VI's `tumvi_512_ds_calib.json` (double-sphere camera model, built for fisheye rigs like this one) and `tumvi_512_config.json` are already shipped.

### 3.2 Run inference

```bash
BASE=~/code/vio_validation_harness
CALIB=~/.local/etc/basalt/tumvi_512_ds_calib.json
CONFIG=~/.local/etc/basalt/tumvi_512_config.json

mkdir -p "$BASE"/results/basalt/corridor4
cd "$BASE"/results/basalt/corridor4   # basalt_vio writes trajectory.txt to the CWD

basalt_vio \
  --dataset-path "$BASE"/data/tumvi/euroc/dataset-corridor4_512_16/ \
  --dataset-type euroc \
  --cam-calib "$CALIB" \
  --config-path "$CONFIG" \
  --marg-data corridor4_marg_data \
  --show-gui 1 \
  --save-trajectory tum \
  --save-groundtruth 1 \
  --result-path corridor4_ate.txt

# Rename file for clarity
mv trajectory.txt trajectory_basalt.txt
```

- `--show-gui 1` opens Basalt's Pangolin viewer — drop to `0` if you want headless.
- `--save-trajectory tum` writes `trajectory.txt` into the current directory (this is why we `cd` first — it's not a path argument).
- `--save-groundtruth 1` also writes `groundtruth.txt` from the same `mocap0` data, as a second cross-check source.
- `--result-path` makes Basalt compute its own RMSE ATE directly.

### 3.3 Output

```
results/basalt/corridor4/
  trajectory_basalt.txt        # timestamp tx ty tz qx qy qz qw
  groundtruth.txt
  corridor4_ate.txt
  corridor4_marg_data/
```

---

## 4. ORB-SLAM3: install and run

Unlike OpenVINS, this uses ORB-SLAM3's **standalone TUM-VI example binary** (not a ROS node), so none of the ROS 2 Jazzy header-compatibility work from §2 applies here — it's a plain CMake build.

### 4.1 Install

```bash
sudo apt install libeigen3-dev libopencv-dev libboost-all-dev libssl-dev

mkdir -p orbslam3_ws && cd orbslam3_ws
# Pangolin (not reliably apt-packaged — build from source)
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && ./scripts/install_prerequisites.sh recommended
cmake -B build && cmake --build build -j$(nproc)
sudo cmake --build build --target install

# ORB-SLAM3
cd ~/code/vio_validation_harness/orbslam3_ws
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
```

ORB-SLAM3's `CMakeLists.txt` still targets C++11, which fails to compile against a lot of current toolchains (very likely to bite on the same Ubuntu 24.04 install where OpenVINS needed the `.hpp` patching in §2) — bump it to C++14 before building:

```bash
sed -i 's/++11/++14/g' CMakeLists.txt
chmod +x build.sh
./build.sh
```

If it fails on `#include <Eigen/...>` not being found, that's a known issue on newer Ubuntu Eigen packaging — swap those includes for `#include <eigen3/Eigen/...>` in the file(s) the compiler flags and rebuild.

This produces `lib/libORB_SLAM3.so` and the example binaries under `Examples/`.

### 4.2 Run inference

ORB-SLAM3 ships everything TUM-VI-specific already — settings YAML, per-sequence timestamp files, and IMU association files — no calibration work needed, same as Basalt:

```bash
BASE=~/code/vio_validation_harness
cd "$BASE"/orbslam3_ws/ORB_SLAM3
mkdir -p "$BASE"/results/orbslam3/corridor4
cd "$BASE"/results/orbslam3/corridor4   # output files land in the CWD

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/code/vio_validation_harness/orbslam3_ws/Pangolin/build/

"$BASE"/orbslam3_ws/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_tum_vi \
  "$BASE"/orbslam3_ws/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  "$BASE"/orbslam3_ws/ORB_SLAM3/Examples/Stereo-Inertial/TUM-VI.yaml \
  "$BASE"/data/tumvi/euroc/dataset-corridor4_512_16/mav0/cam0/data \
  "$BASE"/data/tumvi/euroc/dataset-corridor4_512_16/mav0/cam1/data \
  "$BASE"/orbslam3_ws/ORB_SLAM3/Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt \
  "$BASE"/orbslam3_ws/ORB_SLAM3/Examples/Stereo-Inertial/TUM_IMU/dataset-corridor4_512.txt \
  corridor4_stereoi

# Post-process the results file to scale the timestamps from nanoseconds down to seconds
awk '{printf "%.9f", $1 / 1000000000; for(i=2; i<=NF; i++) printf " %s", $i; print ""}' f_corridor4_stereoi.txt > trajectory_orbslam3.txt
```

Stereo-Inertial to match OpenVINS's and Basalt's stereo+IMU configuration — parity across all three for the comparison.

### 4.3 Output

The last argument (`corridor4_stereoi`) is a filename prefix, not a full path. ORB-SLAM3 writes, into the current directory:

```
results/orbslam3/corridor4/
  f_corridor4_stereoi.txt     # full per-frame trajectory, TUM format
  trajectort_orbslam3.txt     # full per-frame trajectory, TUM format, Timestamps in seconds — use this one
  kf_corridor4_stereoi.txt    # keyframes only, sparser
```

Both are already `timestamp tx ty tz qx qy qz qw` — no conversion needed. Use `f_corridor4_stereoi.txt` for parity with OpenVINS/Basalt's per-frame output.

**Ground truth caveat is stronger here than for the other two**: since ORB-SLAM3 does full SLAM (loop closure, relocalization, map reuse) rather than pure odometry, and corridor4's ground truth only covers the start/end, the ATE you get is really measuring accumulated drift over the whole run, not a true full-trajectory error — same caveat as OpenVINS/Basalt, just worth being extra aware of here since ORB-SLAM3's loop closure won't have anything to close against mid-sequence.

---

## 5. Visualize and validate with `evo`

```bash
BASE=~/code/vio_validation_harness
pip install evo --upgrade --no-binary evo
mkdir -p "$BASE"/results/gt/corridor4

evo_traj euroc "$BASE"/data/tumvi/euroc/dataset-corridor4_512_16/mav0/mocap0/data.csv --save_as_tum
mv data.tum "$BASE"/results/gt/corridor4/trajectory_ground_truth.txt
```

Overlay all three against ground truth:

```bash
cd "$BASE"
evo_traj tum \
  results/openvins/corridor4/trajectory_openvins.txt \
  results/basalt/corridor4/trajectory_basalt.txt \
  results/orbslam3/corridor4/trajectory_orbslam3.txt \
  --ref results/gt/corridor4/trajectory_ground_truth.txt \
  -a --plot_mode xyz --save_plot results/corridor4_traj_comparison.pdf
```

Quantitative ATE, one per system:

```bash
evo_ape tum results/gt/corridor4/trajectory_ground_truth.txt results/openvins/corridor4/trajectory_openvins.txt \
  -a --save_results results/openvins_corridor4_ape.zip

evo_ape tum results/gt/corridor4/trajectory_ground_truth.txt results/basalt/corridor4/trajectory_basalt.txt \
  -a --save_results results/basalt_corridor4_ape.zip

evo_ape tum results/gt/corridor4/trajectory_ground_truth.txt results/orbslam3/corridor4/trajectory_orbslam3.txt \
  -a --save_results results/orbslam3_corridor4_ape.zip
```

`--save_results` zips are self-contained JSON+data (ATE RMSE/mean/median/std) — ready for a downstream validator.

---

## Common pitfalls

- **OpenVINS ROS 2 header mismatches**: the conditional `.hpp`/`.h` patch in §2.1 covers the packages known to trip this up; if colcon fails on a different header, the same `#if __has_include(...)` pattern applies.
- **Basalt `libbasalt.so` not found**: the installer places it in `~/.local/lib`, which isn't on the loader path by default — the `LD_LIBRARY_PATH` export in §3.1 fixes it. Symptom without it: `basalt_vio: error while loading shared libraries: libbasalt.so.x: cannot open shared object file`.
- **Basalt `--save-trajectory tum` writes to the CWD, not a path**: forgetting to `cd` into the per-run output folder first means `trajectory.txt` lands wherever you happened to run the command from.
- **ORB-SLAM3 C++11 build failures**: extremely common on any reasonably current toolchain — the `sed -i 's/++11/++14/g' CMakeLists.txt` step in §4.1 is close to mandatory, not optional, on Ubuntu 24.04.
- **ORB-SLAM3 output prefix confusion**: the final CLI argument is a filename prefix, not a path — files always land in the CWD, hence `cd`-ing into the results folder first in §4.2.
- **corridor4 ground truth is partial**: all three systems' ATE only reflects the overlapping start/end timestamps — expected for this sequence family, not a bug in any of the three.