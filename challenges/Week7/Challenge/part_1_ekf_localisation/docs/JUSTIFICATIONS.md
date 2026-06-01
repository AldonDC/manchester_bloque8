# Part 1 — EKF Localisation: Justifications

This document explains every engineering decision behind the EKF-based
localisation pipeline, as required by the Final Challenge PDF (Part 1,
"The student must explain the reasoning behind every engineering decision
made").

---

## 1. ArUco dictionary: `DICT_4X4_50`

**Decision.** We use the `4x4_50` dictionary from OpenCV's ArUco module.

**Why not larger (e.g. `5x5_100`).**
- The simulated camera publishes 640×480 at ~30 Hz. At a 0.18 m marker
  side and 1.5 m typical detection range, a 4×4 marker occupies ~58×58 px;
  a 5×5 marker at the same range would occupy ~58 px split into 25 cells
  instead of 16, dropping below the per-cell sampling threshold OpenCV
  recommends (≥ 4 px/cell).
- We only need 4 unique IDs (one per landmark in our arena). `4x4_50` is
  smaller, has higher minimum Hamming distance per bit budget, and is
  more robust under partial occlusion and motion blur.

**Why not lower (e.g. `4x4_50` with side 0.10 m).** A smaller physical
size would force the robot to be closer than 0.8 m to detect — that
violates the arena layout (the corridor width is ~1 m).

---

## 2. Number and placement of markers

**Decision.** 4 markers, placed on the **inner faces** of the central
dividing walls `ih2_left` and `ih2_right` (at `y = 2.0`), so that:

| ID | (x, y) | facing | visible from |
|----|--------|--------|--------------|
| 1  | (0.6, 1.92) | south (−Y) | south corridor, looking N |
| 2  | (4.4, 1.92) | south (−Y) | south corridor, looking N |
| 0  | (0.6, 2.08) | north (+Y) | north corridor, looking S |
| 3  | (4.4, 2.08) | north (+Y) | north corridor, looking S |

**Why this layout.**
1. **Coverage of both halves.** South and north corridors each see two
   markers, so the robot is never more than a 180° pivot away from a
   correction opportunity.
2. **Range-bearing diversity.** Markers are widely spaced laterally
   (`Δx = 3.8 m`), which gives the EKF a strong baseline for triangulation
   when both are visible — the Jacobian H is well-conditioned.
3. **No marker behind the robot's blind side.** The Puzzlebot camera
   has ~60° HFOV pointing forward; we placed the markers along the
   intended trajectory so the robot does not need to look backward to
   see them.

**Why 4 and not 2 or 8.**
- 2 markers leaves the arena under-observable: yaw is poorly constrained
  when only one marker is visible (the EKF can rotate the estimate
  around the marker's bearing axis without changing the predicted
  measurement much).
- 8 markers would add false correspondences risk without measurable
  benefit at our arena scale.

---

## 3. Observation model: `range_bearing`

**Decision.** We expose two models in `observation_model.py` and run
`range_bearing` by default.

**`range_bearing`:** `z = [r, φ]^T` where `r` is the Euclidean distance
robot → marker and `φ` is the relative bearing in the robot frame.

**`cartesian`:** `z = [Δx, Δy]^T` of the marker in the robot frame.

**Why range-bearing as default.**
- It matches the **physical measurement geometry** of a monocular ArUco
  detector: OpenCV gives us the marker's translation in the camera
  frame, which we convert to `(r, φ)` directly. Going to Cartesian
  forces an extra rotation that mixes range error and bearing error
  into both components, making the noise covariance `R` harder to
  calibrate.
- The Jacobian `H_rb` is non-zero everywhere except at the singular
  case `r = 0` (robot on top of the marker — impossible physically).
- It is the model used in every textbook treatment of EKF with fiducial
  landmarks; this makes the implementation auditable against published
  derivations.

**When `cartesian` is better.** When the camera is very close to the
marker and the bearing resolution becomes the bottleneck (small lateral
errors cause big bearing changes); we left it as a switch so we can
demo both during the evaluation.

---

## 4. Process noise `k_r = k_l = 0.05`

**Decision.** Per-wheel noise gains both set to `0.05`.

**Why these values.**
- The Puzzlebot's encoder model assumes wheel velocity noise is
  proportional to the commanded wheel speed: `σ_ω = k · |ω|`. So `k_r,
  k_l` directly say "the std-dev of each wheel's measured velocity is
  5% of its magnitude".
- At maximum operating velocity (~0.3 m/s linear, 1.5 rad/s angular),
  the resulting position drift per second of dead-reckoning is ~1.5 cm
  — close to what we observed empirically by integrating without
  corrections for 20 s and comparing to ground truth.
- Setting `k_r = k_l` is justified because both wheels are identical
  motors with identical encoders.

**Validation knob.** If NEES at the end of a run sits well above 3 the
filter is over-confident → bump `k_r/k_l` up. If NEES sits well below 3
the filter is under-confident → bump them down. The final summary on
the terminal prints exactly this so we can tune empirically.

---

## 5. Mahalanobis gate `χ²(2, 0.99) = 9.21`

**Decision.** Reject any observation whose innovation Mahalanobis
distance squared exceeds 9.21.

**Why this threshold.**
- The innovation `y = z - h(x)` is 2-D (range, bearing). For a Gaussian
  filter operating correctly, `y^T S^{-1} y` follows a χ²(2)
  distribution. The 99-th percentile of χ²(2) is 9.21.
- Choosing 99% (not 95%) trades a few percent of correct rejections
  during transients for a much lower false-acceptance rate on
  mis-identified markers — and for an EKF that just lost track,
  accepting one outlier corrupts the belief for many seconds.
- We log every rejection (`/ekf/state → reject:id=X:d2=...`) and count
  them per marker for the final summary, so we can see when the gate
  was actually triggered.

---

## 6. Step saturation in the correction

**Decision.** Clip the Kalman correction step to `±1 m` in (x, y) and
`±0.5 rad` in yaw before applying it.

**Why.**
- We observed (during early integration) that the very first correction
  after a long no-marker period can produce a large `K · y` because the
  covariance `P` has grown unbounded. Without clipping, the RViz arrow
  teleports across the arena and the TF buffer panics ("Detected jump
  back in time").
- The clip is generous enough (1 m in one update at 10 Hz means a
  catch-up of 10 m/s, far above the robot's max speed) that it never
  triggers in normal operation. It only kicks in to **bound the
  worst-case transient**.

---

## 7. Initial covariance `P_0 = 0`

**Decision.** Start with zero covariance — equivalent to "I know the
initial pose exactly".

**Why.**
- The launch passes `x_init / y_init / theta_init` from the same values
  used to spawn the robot in Gazebo. Because both come from the same
  source, the initial pose really *is* exact at `t = 0` from the
  filter's point of view.
- The process noise `Q_k` will start growing `P_k` on its own from the
  first prediction step, so the initial zero is not a numerical
  problem.

---

## 8. Definition of robustness

We define **robustness** as the conjunction of three properties, each
measurable from the metrics the node logs:

1. **Bounded error.** RMSE_xy stays below 0.10 m across the three PDF
   scenarios (multi-marker, no-marker, partial).
2. **Consistent uncertainty.** NEES mean ∈ [2, 4] and at least 80 % of
   samples fall inside the χ²(3) 95 % band [0.216, 9.348]. This
   guarantees the covariance ellipsoid drawn in RViz is statistically
   meaningful.
3. **Outlier immunity.** Mis-identified or stale markers are gated out
   (the count of `rej` updates is non-zero when we artificially feed
   the wrong ID, and zero otherwise).

The corresponding strategies in the code:

- *(1)* range-bearing model + per-marker correction, plus odometry
  fusion at 50 Hz so the gap between corrections is short.
- *(2)* the `k_r, k_l` tuning is done against NEES, not against RMSE
  alone.
- *(3)* the chi² gate + the `aruco_map` membership check (`drop:unmapped`)
  reject IDs the filter does not trust.

---

## 9. Coordinate transformations chain

The full transform chain the perception side uses is:

```
F_m (marker)  --[ArUco pose]-->  F_c (camera optical)
F_c           --[static TF]-->   F_r (base_footprint)
F_r           --[EKF belief]-->  F_world (odom)
```

- `F_c → F_r`: a fixed offset declared at the top of `aruco_detector.py`
  matching the Puzzlebot URDF's camera mount (0.08 m forward, 0.10 m
  up, yaw 0). This is constant for the simulated robot.
- `F_r → F_world`: comes from the EKF state, broadcast every tick as a
  TF (`odom → base_footprint`) so RViz can render the RobotModel with
  the actual belief, not the raw odometry.

We chose **not** to publish a `map → odom` correction (which would be
the SLAM-style frame layout) because Part 1 explicitly assumes the
ArUco map is known — `map` and `odom` are identical and we publish a
static identity TF between them in the launch.

---

## 10. Evaluation metrics

The node logs four kinds of metrics, all of them mandated or strongly
implied by the PDF:

| Metric | What it tells us |
|--------|------------------|
| `err_xy`, `err_yaw` | Instantaneous error vs Gazebo ground truth |
| `trace(P)` | Total uncertainty — must shrink when markers are seen |
| `NEES` | Whether the covariance is well-calibrated |
| `upd / rej` per marker | Which landmarks are actually correcting the filter |

The final summary printed at Ctrl-C includes RMSE_x, RMSE_y, RMSE_yaw,
NEES_mean, NEES_in_band%, accepted updates, rejected updates per ID, and
a one-line interpretation of whether the filter is currently optimistic,
conservative, or consistent.
