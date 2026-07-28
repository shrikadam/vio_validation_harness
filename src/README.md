## Capstone Project: Automated VIO Regression & Fault-Injection Harness

### Phase 1: High-Performance Data Ingestion & The Target

Your first step is to set up the data streams and the target algorithm under test.

* **The Dataset:** Download a sequence from the **TUM-VI** dataset (e.g., the "Room" or "Corridor" sequence). TUM-VI is excellent because it features hardware-synchronized 512 Hz IMU data, 20 Hz stereo images, and $120\text{ Hz}$ motion-capture ground truth.
* **The Target Algorithm:** Clone and compile a production-grade open-source VIO system like **OpenVINS** or **VINS-Mono**. This represents the C++ state estimator you are being paid to test.
* **The Ingestion Module:** Write a high-throughput parser to read the TUM-VI raw data and the output trajectory of the VIO. For maximum execution speed over gigabytes of logs, you can implement the heavy trajectory parsing loops in high-performance C++ and expose them to your Python orchestration framework using `pybind11`.

### Phase 2: The Object-Oriented Evaluation Engine

Build the mathematical core of your validation harness using the architectural patterns discussed on Day 4.

* **Implement the Strategy Pattern:** Code independent Python modules for computing Absolute Trajectory Error (ATE) and Relative Pose Error (RPE).
* **Implement $SE(3)$ Alignment:** Write the Umeyama algorithm from scratch to dynamically align the VIO's arbitrary starting frame to the TUM-VI motion-capture room origin.
* **Temporal Synchronization:** The VIO output poses will likely not align perfectly with the $120\text{ Hz}$ ground-truth timestamps. Implement a linear interpolation module in $SE(3)$ (using Spherical Linear Interpolation, or SLERP, for the $SO(3)$ quaternion components) to match the timestamps before computing the error.

### Phase 3: "Staff-Level" Hardware Fault Injection

This is the crown jewel of the project. A validation engineer doesn't just test perfect data; they test how the algorithm handles hardware degradation. Write a Python "Tamper Module" that intercepts the TUM-VI data *before* it hits the VIO, injecting specific, mathematically grounded errors.

* **Test Case 1: Temporal Desynchronization (The Time Sync Bug)**
* *Action:* Artificially shift all IMU timestamps by $+15\text{ ms}$ in the data stream.
* *Expected Result:* Your evaluation engine should flag a massive spike in RPE during rapid head rotations, while slow movements remain relatively unaffected.


* **Test Case 2: Extrinsic Thermal Shift**
* *Action:* Modify the $T_{cam}^{imu}$ calibration matrix by injecting a $0.5^\circ$ rotation, simulating the camera lens physically warping due to the headset getting hot.
* *Expected Result:* The VIO should slowly accumulate scale drift and global ATE over time.


* **Test Case 3: IMU Bias Instability**
* *Action:* Inject a subtle, drifting random-walk bias into the raw $512\text{ Hz}$ gyroscope data array.
* *Expected Result:* Your dashboard should detect cubic positional error growth $\mathcal{O}(t^3)$ if the VIO's internal factor graph fails to estimate and reject the new bias.

### Phase 4: CI/CD Mockup and KPI Dashboarding

Wrap the entire system into a testing framework that would run automatically on a server.

* **Pytest Integration:** Parameterize your test suites using Pytest. Set strict failure thresholds (e.g., `assert rmse_ate < 0.05 meters` and `assert max_latency < 20 ms`).
* **Visual Analytics:** Generate an automated PDF or HTML report upon completion. Plot the 3D ground-truth vs. estimated trajectories, and create a heat map showing exactly *where* in the room the tracking lost confidence or jittered.