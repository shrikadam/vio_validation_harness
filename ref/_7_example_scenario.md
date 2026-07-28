## The Scenario: *"The Case of the Wobbling Whip-Pan"*

> **Problem Statement:**
> The XR Runtime team files a critical regression ticket: *"During fast head turns ('whip-pans') while playing a high-motion VR game, the virtual environment violently flickers for 100ms, followed by a slow 'swimming' effect where the virtual floor tilts by ~2° and drifts 15cm over the next 5 seconds. This causes immediate motion sickness and breaks the physical room boundary."*

As a Staff Engineer, you are tasked with owning the end-to-end investigation: defining the test strategy, ingesting the logs, isolating whether this is an algorithm bug, a calibration issue, or a pose extrapolation delay, and building automated Python tooling to prevent it from happening again.

---

## 1. The Inputs You Receive

When you start analyzing this issue, you are handed four primary data streams:

```
                  +-----------------------------------+
                  |        Raw Headset Sensors        |
                  |  - Dual Grayscale Cameras (60Hz)  |
                  |  - High-Freq IMU (1000Hz)         |
                  +-----------------+-----------------+
                                    |
                                    v
+-----------------------+   +---------------+   +------------------------+
| Vicon Mocap Room Logs |-->|      DATA     |<--| Display Pipeline Logs  |
| - Ground Truth (GT)   |   |   PROCESSING  |   | - Extrapolated Poses   |
|   at 200Hz            |   |    PIPELINE   |   |   for ATW (90-120Hz)   |
+-----------------------+   +---------------+   +------------------------+
                                    ^
                                    |
                  +-----------------+-----------------+
                  |      Calibration & Configs        |
                  |  - Extrinsics (Camera-to-IMU)     |
                  |  - GTSAM Factor Graph Priors      |
                  +-----------------------------------+

```

1. **Raw Sensor Logs (Binary / ROS Bags):**
* High-frequency IMU readings: Raw gyro $\tilde{\omega}$ (rad/s) and accelerometer $\tilde{a}$ (m/s²).
* Grayscale tracking camera frames (~60Hz) with exposure timestamps.
* VIO State Outputs ($t_0$): Optimized poses $q_0, p_0, v_0$ and IMU biases $b_g, b_a$.


2. **Display & Extrapolation Logs:**
* High-frequency predicted poses ($q_{\text{future}}, p_{\text{future}}$) published to Asynchronous TimeWarp (ATW) at display refresh rates (90Hz / 120Hz) over a $\Delta t = 15\text{ms}$ motion-to-photon prediction horizon.


3. **Ground Truth (GT) Reference Logs:**
* millimeter-accurate 6DoF poses ($q_{\text{gt}}, p_{\text{gt}}$) captured at 200Hz in an OptiTrack/Vicon motion capture room.


4. **Calibration & Configuration Artifacts:**
* Camera intrinsic matrices ($K$), Camera-to-IMU extrinsic transform ($T_{\text{cam}}^{\text{imu}}$), and factor graph covariance settings.



---

## 2. What You Are Expected To Do (Step-by-Step)

### Step 1: Spatial & Temporal Log Alignment

Before you can run a single metric, your Python pipeline must eliminate setup noise:

* **Time Sync (Cross-Correlation):** You compute the cross-correlation of angular velocity profiles ($\Vert{}\omega\Vert{}$) between the headset IMU and the Vicon ground truth to solve for clock drift and align timestamps down to sub-millisecond precision.
* **Hand-Eye Calibration ($AX = XB$):** The Vicon tracks reflective markers on the outer shell, while VIO tracks the internal IMU center. You solve $AX = XB$ to transform the Vicon ground truth into the headset's internal frame.
* **Trajectory Alignment:** You run **Umeyama $Sim(3)$ alignment** (like `evo` does) to map the arbitrary initial VIO origin to the absolute world origin of the Mocap room.

---

### Step 2: Signal Decomposition (Jitter vs. Drift)

You write a Python script using `numpy` and `scipy` to decompose the overall error signal into its two distinct physical components:

$$\text{Error}(t) = \text{Jitter}(t) + \text{Drift}(t)$$

```
                                 ERROR SIGNAL DECOMPOSITION
   
   Raw Pose Error (Total)              High-Pass Filter               Jitter (RPE over 10ms)
  +-----------------------+          +-------------------+          +-----------------------+
  |  Spikes + Parabola    | -------> |  Isolates Noise   | -------> | Fast Flickering Noise |
  +-----------------------+          +-------------------+          +-----------------------+
              |                                                                 
              |                        Low-Pass Filter                Drift (ATE over 5s)
              |                      +-------------------+          +-----------------------+
              +--------------------> | Isolates Trend    | -------> | Quadratic Arc         |
                                     +-------------------+          +-----------------------+

```

1. **Jitter Isolation (Relative Pose Error - RPE):**
* You evaluate RPE over short $10\text{ms} - 30\text{ms}$ sliding windows.
* **The Diagnosis:** You discover massive RPE spikes during the 100ms window of the whip-pan. The high-frequency noise causes the virtual image to vibrate violently.


2. **Drift Isolation (Absolute Trajectory Error - ATE):**
* You evaluate ATE over $1\text{s} - 10\text{s}$ windows.
* **The Diagnosis:** The trajectory smoothly diverges along a quadratic parabolic curve immediately after the whip-pan finishes.



---

### Step 3: Root-Cause Failure Analysis (Applying the Math)

Now you dig into the physics of why the perception system failed during this motion:

#### A. Diagnosing the Jitter (Visual Feature Loss & Saturation)

* **What happened:** During the whip-pan, image frames suffered severe motion blur. The optical flow tracker lost tracking on 90% of visual features.
* **Extrapolation Failure:** To compensate, ATW relied entirely on IMU dead-reckoning extrapolation:

$$q_{\text{future}} = q_0 \otimes \exp\left(\frac{1}{2}\omega \Delta t\right)$$



Because the angular velocity $\omega$ was changing rapidly, simple 1st-order integration introduced high-frequency prediction errors, manifesting as **jitter** on the display.

#### B. Diagnosing the Drift (Gravity Bleed)

* **What happened:** The rapid rotation caused the physical IMU gyroscope to briefly hit its physical ceiling (saturation, e.g., $>2000^\circ/\text{s}$).
* **Orientation Error:** The SLAM state estimator miscalculated the post-rotation orientation $q_0$ by just $0.8^\circ$ of pitch.
* **The Gravity Bleed Equation:** When rotating proper acceleration into world coordinates, the $0.8^\circ$ tilt error caused a fraction of the $9.81\text{m/s}^2$ gravity vector to bleed into horizontal acceleration:

$$a_{\text{world}} = (q_0 \otimes a_{\text{imu}} \otimes q_0^{-1}) - g$$



Integrating this parasitic acceleration twice over time produced the quadratic translational drift you observed:

$$p_{\text{drift}}(t) = \frac{1}{2} a_{\text{bleed}} t^2$$



---

### Step 4: Factor Graph Residual & Covariance Inspection

To prove this to the core algorithm engineering team, you inspect the optimizer state in GTSAM / Ceres:

* You extract the **Marginal Covariance Matrix** ($\Sigma$) from the factor graph.
* You show that when visual reprojection factors dropped out due to motion blur, the uncertainty covariance for orientation exploded.
* **The Key Insight:** The SLAM system *knew* its confidence dropped, but it failed to inform the downstream pose extrapolator to smoothly damp the prediction, causing a harsh step-change in tracking.

---

### Step 5: Automation & KPI Gatekeeping (The Deliverable)

At the Staff Engineer level, you don't just solve the bug once—you build the automated testing infrastructure to keep it fixed forever:

1. **Python KPI Suite:** You write a headless Python validation tool integrated into Qualcomm’s CI/CD pipeline.
2. **Automated Thresholds:**
    * **Max allowable Jitter (RPE @ 20ms):** $< 0.5\text{mm}$
    * **Max allowable Drift Rate (ATE / time):** $< 1\text{cm/sec}$
    * **Prediction Horizon Error (at 15ms horizon):** $< 1.5\text{mm}$


3. **Automated Actionable Reports:** When a nightly build regression occurs, your tool automatically generates an HTML diagnostic dashboard containing:
    * 3D overlay plots of GT vs. Estimated Trajectories (`evo` style).
    * Time-series plots showing exact correlation between Gyro Saturation $\to$ Orientation Covariance Spike $\to$ Gravity Bleed Drift.



---

## Summary of the Engineering Cycle

| Stage | Input Used | Mathematical / Analytical Concept | Outcome / Deliverable |
| --- | --- | --- | --- |
| **1. Setup** | Mocap Logs + Headset Logs | Time sync cross-correlation, Hand-Eye $AX=XB$, Umeyama $Sim(3)$ | Unified, perfectly aligned reference frame |
| **2. Triage** | Headset Trajectory | RPE (sliding window) vs. ATE decomposition | Isolated Jitter spike from smooth Drift curve |
| **3. Deep Dive** | High-freq IMU + Display Poses | Kinematic pose extrapolation, $SO(3)$ rotation, Gravity Bleed math | Identified IMU gyro saturation + $0.8^\circ$ tilt bias |
| **4. Backend** | GTSAM / Ceres logs | Marginal Covariance ($\Sigma$) & Residual Analysis | Proved factor graph uncertainty wasn't gated |
| **5. Deliverable** | Python + `pandas` + `evo` | Automated metric regression pipelines | CI/CD gatekeeper script blocking bad code merges |

---