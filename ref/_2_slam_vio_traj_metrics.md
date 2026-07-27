For Day 2, we are bridging the gap between raw tracking data and actionable performance metrics. If Day 1 was about understanding a single point in space, Day 2 is about understanding a continuous path.

Here is the breakdown of SLAM trajectory alignment and evaluation, followed by your Python demonstrator.

---

## 1. The Core Problem: Why Do We Need Alignment?

When you fire up a VIO (Visual-Inertial Odometry) system on an XR headset, it sets its origin $(0,0,0)$ right where it turned on. However, your ground-truth motion capture room (like Vicon) has its origin permanently bolted to the corner of the physical room.

Before you can compare the estimated trajectory to the ground truth, you must mathematically move and rotate the VIO trajectory into the Vicon room's coordinate frame. This is called **Trajectory Alignment**.

### The Umeyama Algorithm

The industry standard for this is the **Umeyama algorithm**. It solves for the optimal transformation—Rotation ($R$), Translation ($t$), and Scale ($s$)—that aligns two sets of 3D point clouds.

* **For Stereo/Depth SLAM:** The scale is known (real-world metric scale), so we constrain $s=1$ and solve for an $SE(3)$ transformation.
* **For Monocular SLAM:** The scale is unobservable (the system doesn't know if it's looking at a small toy close up or a real building far away). Here, we solve for a $Sim(3)$ (Similarity) transformation where $s$ is actively calculated.

---

## 2. ATE (Absolute Trajectory Error)

**What it measures:** Global consistency. Did the trajectory stay true to the exact shape of the ground truth over the entire run?
**How it works:** After aligning the trajectories using Umeyama, ATE compares every estimated pose directly against the ground-truth pose at the exact same timestamp.

If $Q_i$ is the ground truth pose and $P_i$ is the estimated pose at time $i$, and $S$ is the alignment transformation, the error at step $i$ is:


$$E_i = Q_i^{-1} S P_i$$

You then take the translational component of $E_i$ and calculate the **Root Mean Square Error (RMSE)** across all frames. ATE is highly sensitive to early rotation errors (a 1-degree error at the start causes massive positional error 100 meters later).

---

## 3. RPE (Relative Pose Error)

**What it measures:** Local drift. How much did the algorithm mess up *over a specific time or distance interval* (e.g., drift per 1 second, or drift per 1 meter)?
**How it works:** Instead of comparing poses globally, RPE looks at the *change* in pose ($\Delta$) between time $i$ and time $i + \Delta t$.

It compares the motion estimated by the VIO against the motion recorded by the ground truth:


$$E_i = (Q_i^{-1} Q_{i+\Delta})^{-1} (P_i^{-1} P_{i+\Delta})$$

RPE is crucial for XR because a headset might slowly drift across a room (bad ATE, but user doesn't notice), but if it jumps 5 centimeters in 0.1 seconds (bad RPE), the user will immediately feel motion sick.
