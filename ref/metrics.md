Yes, both **drift** and **jitter** can be quantified precisely using signal processing and geometric trajectory analysis.

While people often lump them together as "tracking error," they operate on opposite ends of the frequency spectrum: **drift** is a low-frequency, cumulative systematic error, whereas **jitter** is high-frequency, stochastic spatial noise.

---

## 1. Quantifying Drift (Low-Frequency Accumulated Error)

Drift measures how the estimated trajectory gradually strays from reality over time or distance.

### A. Relative Pose Error (RPE) over Distance Windows ($L$)

Rather than looking at single frame-to-frame steps, standard VIO benchmarks (like KITTI or EuRoC) compute the mean translation/rotation error over fixed path segments of varying lengths $L$ (e.g., $10\text{m}, 20\text{m}, 50\text{m}$).

For a sub-segment starting at frame $i$ and ending at frame $i + \Delta k$ corresponding to distance $L$:

$$E_{\text{RPE}}(i, L) = \left\Vert{} (\mathbf{p}_{\text{gt}, i+\Delta k} - \mathbf{p}_{\text{gt}, i}) - \mathbf{R}_{\text{gt}, i}^\top (\mathbf{p}_{\text{est}, i+\Delta k} - \mathbf{p}_{\text{est}, i}) \right\Vert{}$$

Plotting $E_{\text{RPE}}(L)$ against segment length $L$ shows your drift curve.

### B. Percentage Drift Rate

To state drift as a single benchmark metric, express the accumulated Absolute Trajectory Error (ATE) as a percentage of the total distance traveled ($D_{\text{total}}$):

$$\text{Drift Rate}_{\text{dist}} = \frac{\text{ATE}_{\text{final}}}{D_{\text{total}}} \times 100\%$$

*Example:* A VIO system that accumulates $0.15\text{m}$ of ATE after traversing $30\text{m}$ of corridor has a **$0.5\%$ drift rate**.

### C. Temporal Drift Rate

To isolate time-dependent sensor degradation (like IMU gyro bias instability during zero-motion periods), fit a line to the ATE over time:

$$\text{Drift Rate}_{\text{time}} = \frac{d}{dt} \text{ATE}(t) \quad \left[\text{m/s}\right]$$

---

## 2. Quantifying Jitter (High-Frequency Noise & Instability)

Jitter measures high-frequency frame-to-frame wobble or vibration—the "shimmer" or "bounce" a user experiences when standing completely still or moving smoothly.

### A. High-Pass Filtered Residual RMS

Subtract the ground truth trajectory from the estimated trajectory to get the spatial residual vector $\mathbf{r}(t) = \mathbf{p}_{\text{est}}(t) - \mathbf{p}_{\text{gt}}(t)$. Pass $\mathbf{r}(t)$ through a high-pass filter (e.g., Butterworth with a cutoff at $f_c = 5\text{ Hz}$) to remove low-frequency motion and isolate jitter:

$$\mathbf{r}_{\text{jitter}}(t) = \text{HighPassFilter}(\mathbf{r}(t), f_c)$$

Then compute the Root Mean Square (RMS) of the isolated jitter:

$$\text{Jitter}_{\text{RMS}} = \sqrt{\frac{1}{N} \sum_{k=1}^N \left\Vert{} \mathbf{r}_{\text{jitter}}(k) \right\Vert{}^2} \quad \left[\text{m}\right]$$

### B. Discrete Jerk Residual (3rd Derivative)

In human perception (especially for XR/VR headsets), jitter is felt as unnatural changes in acceleration. Jerk is the third derivative of position.

For discrete pose measurements at timestep $\Delta t$, compute the jerk of the tracking error signal:

$$\mathbf{j}_k = \frac{\mathbf{r}_{k+1} - 3\mathbf{r}_k + 3\mathbf{r}_{k-1} - \mathbf{r}_{k-2}}{\Delta t^3}$$

The standard deviation of this error jerk ($\sigma_{\mathbf{j}}$ in $\text{m/s}^3$) gives a direct quantitative index of high-frequency jitter.

### C. Spectral Power Density (PSD / FFT)

Convert the tracking error signal $\mathbf{r}(t)$ into the frequency domain using a Fast Fourier Transform (FFT).

Integrate the Power Spectral Density (PSD) over human-perceptible jitter bands (typically $5\text{ Hz} \le f \le 30\text{ Hz}$):

$$P_{\text{jitter}} = \int_{f_{\text{low}}}^{f_{\text{high}}} \text{PSD}_{\mathbf{r}}(f) \, df$$

---

## Summary of Metric Implementations

| Metric | Target Phenomenon | Typical Calculation | Target Benchmark Units |
| --- | --- | --- | --- |
| **Drift (Spatial)** | Accumulated scale/heading error | $\text{RPE} / L$ or $\frac{\text{ATE}_{\text{final}}}{D_{\text{total}}}$ | $\% \text{ of traveled distance}$ (e.g., $0.2\%$) |
| **Drift (Temporal)** | IMU bias drift over static periods | Linear slope of $\text{ATE}(t)$ | $\text{mm/s}$ or $\text{m/hr}$ |
| **Jitter (Magnitude)** | High-frequency position wobble | RMS of high-pass filtered error ($f > 5\text{ Hz}$) | $\text{mm RMS}$ |
| **Jitter (Perceptual)** | Rapid shaking / motion instability | Standard deviation of 3rd derivative (Jerk) | $\text{m/s}^3$ |