Here is your **80/20 Study Curriculum**—the exact 20% of high-yield concepts that will drive 80% of your interview performance, tailored to your 6-day sprint.

---

## Part 1: The 80/20 Study Curriculum (6-Day Sprint)

```
        ┌──────────────────────────────────────────────────────────┐
        │                 6-DAY SPRINT AT A GLANCE                 │
        ├─────────┬────────────────────────────────────────────────┤
        │ Day 1   │ 3D Math Fundamentals & Pose Representation     │
        │ Day 2   │ SLAM/VIO Error Metrics (ATE/RPE) & Alignment   │
        │ Day 3   │ Head Tracking Specific KPIs & Latency          │
        │ Day 4   │ Python Architecture for Perception Pipelines   │
        │ Day 5   │ Root-Cause Debugging & Sensor Calibration      │
        │ Day 6   │ System Design Mocking & Technical Narrative    │
        └─────────┴────────────────────────────────────────────────┘

```

---

### Day 1: 3D Math & Pose Representation

* **Rotations & $SO(3)$ Representation:** Quaternions (unit norm constraints, SLERP, quaternion multiplication), Rotation Matrices, Axis-Angle, Euler Angles (Gimbal lock).
* **Transformation Matrices & $SE(3)$:** Rigid body transformations in 3D.
* **Distance Metrics on $SO(3)$:** Calculating angular distance between two rotations ($\theta = \arccos\left(\frac{\text{Tr}(R_1 R_2^T) - 1}{2}\right)$ or relative quaternion inner product).
* *Task:* Code a Python function from scratch using `NumPy` to calculate geodesic rotational error between two array streams of quaternions.

### Day 2: SLAM / VIO Trajectory Metrics & Alignment

* **ATE (Absolute Trajectory Error):** Measures global consistency.
* **RPE (Relative Pose Error):** Measures local drift per unit time / distance (drift rate).
* **Trajectory Alignment (Umeyama Algorithm):** Solving rigid $SE(3)$ or Similarity $Sim(3)$ alignment between estimated trajectory and ground-truth motion capture (Vicon/OptiTrack) using Singular Value Decomposition (SVD).
* *Task:* Study the internals of the `evo` package (`evo_ape` and `evo_rpe`). Understand how timestamp association works (nearest neighbor / linear interpolation).

### Day 3: XR Head Tracking KPIs & Motion Prediction

* **XR Specific Failure Modes:** Jitter (high-frequency noise while stationary), Drift (low-frequency systematic offset), Slip / Swim (when virtual objects move relative to physical world).
* **Predictive Tracking:** EKF-based state extrapolation for motion-to-photon latency compensation ($t + \Delta t$).
* **Edge Cases:** Fast angular acceleration (head flicks), dark environments, low-texture surfaces, camera occlusion, timestamp desynchronization between camera exposure and IMU ticks.

### Day 4: Python Architecture for Perception Pipelines

* (See **Part 2** below for patterns to master).

### Day 5: Sensor Calibration & Debugging Workflows

* **Camera Intrinsic/Extrinsic Models:** Pinhole model, Kannala-Brandt (Fisheye), radial/tangential distortion.
* **IMU Calibration:** Allan Variance (gyroscope/accelerometer white noise vs. random walk bias), spatial extrinsic offset ($T_{\text{cam}}^{\text{imu}}$), temporal offset (time latency between IMU clock and camera trigger).

### Day 6: Practice Technical Scenarios

* Synthesize 3 concrete stories from your past experience at Airbus/Citifai using the **STAR-V** method:
* **S**ituation
* **T**ask
* **A**ction
* **R**esult
* **V**alidation (How did you mathematically prove it worked?)



---

## Part 2: Python Design & Architectural Patterns for Perception QA

In a Staff Engineer interview, writing simple Python scripts isn't enough—they want to see **clean, maintainable, extensible software architecture** for processing massive datasets and orchestrating hardware test runs.

Sharpen these 5 specific patterns:

---

### 1. The Pipeline / Chain of Responsibility Pattern

**Use Case:** Modular offline log processing. Multi-sensor tracking logs must pass through sequential transformation stages (e.g., Parsing $\rightarrow$ Timestamp Sync $\rightarrow$ Ground-Truth Alignment $\rightarrow$ Metric Computation $\rightarrow$ Report Generation).

```python
from abc import ABC, abstractmethod
from typing import Dict, Any

class PipelineStage(ABC):
    def __init__(self):
        self._next_stage = None

    def set_next(self, stage: 'PipelineStage') -> 'PipelineStage':
        self._next_stage = stage
        return stage

    @abstractmethod
    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        if self._next_stage:
            return self._next_stage.process(context)
        return context

class TimestampSyncStage(PipelineStage):
    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        # Perform time-association between VIO trajectory and GT
        context['synced_trajectories'] = ... 
        return super().process(context)

```

### 2. Strategy Pattern

**Use Case:** Swappable metric engines. The test harness needs to dynamically apply different error calculation strategies depending on whether the test is evaluating 6DoF head pose, 3DoF rotational tracking, or 3D reconstruction quality.

```python
from abc import ABC, abstractmethod
import numpy as np

class EvaluationStrategy(ABC):
    @abstractmethod
    def evaluate(self, est_poses: np.ndarray, gt_poses: np.ndarray) -> dict:
        pass

class ATETranslationStrategy(EvaluationStrategy):
    def evaluate(self, est_poses: np.ndarray, gt_poses: np.ndarray) -> dict:
        # Compute RMSE of Euclidean distances
        errors = np.linalg.norm(est_poses[:, :3, 3] - gt_poses[:, :3, 3], axis=1)
        return {"rmse": np.sqrt(np.mean(errors**2)), "max": np.max(errors)}

class GeodesicRotationStrategy(EvaluationStrategy):
    def evaluate(self, est_poses: np.ndarray, gt_poses: np.ndarray) -> dict:
        # Compute SO(3) rotational differences
        pass

```

### 3. Builder / Factory Pattern

**Use Case:** Constructing test datasets and complex test configurations. Automated test suites often run hundreds of test configurations (varying light levels, motion speeds, sensor frequencies).

```python
class TestScenarioBuilder:
    def __init__(self):
        self.config = {}

    def set_lighting(self, lux_level: int):
        self.config['lighting'] = lux_level
        return self

    def set_motion_profile(self, profile_type: str):
        # e.g., "fast_head_rotation", "slow_walking"
        self.config['motion'] = profile_type
        return self

    def build(self) -> 'TestScenario':
        return TestScenario(self.config)

```

### 4. Pytest Fixtures & Parametrization (Framework Native Pattern)

**Use Case:** Building scalable regression test suites in Python. Qualcomm will expect you to write clean Pytest architectures using parameterized fixtures for large log suites.

```python
import pytest

@pytest.fixture(scope="session")
def load_ground_truth():
    # Load motion capture dataset once
    return load_vicon_data("logs/gt.tum")

@pytest.mark.parametrize("log_file, max_allowed_drift_m_per_s", [
    ("normal_walk.bag", 0.01),
    ("head_flick.bag", 0.05),
    ("low_light.bag", 0.03),
])
def test_head_tracking_drift(log_file, max_allowed_drift_m_per_s, load_ground_truth):
    vio_output = run_vio_pipeline(log_file)
    drift = compute_rpe(vio_output, load_ground_truth)
    assert drift < max_allowed_drift_m_per_s

```

### 5. Producer-Consumer / Async Queue Pattern

**Use Case:** High-throughput streaming log evaluation. Parsing massive raw camera feeds (60–120 FPS) and IMU packets (1000 Hz) concurrently requires non-blocking streaming queues (`asyncio.Queue` or `multiprocessing`).

---

## Pro-Tip for the Interview

When asked how you build validation tooling, always mention **data abstraction, reproducibility, and failure isolation**:

> *"I design validation pipelines where data loading is completely decoupled from metric calculation via the Strategy Pattern. This allows us to plug in new ground-truth formats—like OptiTrack, synthetic Blender simulations, or Vicon—without breaking our core KPI analysis logic."*