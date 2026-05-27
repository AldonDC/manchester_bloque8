# Evaluation Metrics — Part 1 EKF

The PDF asks: *"the student must define evaluation metrics of their
algorithm to verify robustness"*. Here is what we measure and how.

## 1. Absolute Trajectory Error (ATE)

For each EKF tick `k` we have
- the EKF belief mean `x_k = (x, y, θ)`
- the ground truth from Gazebo on `/ground_truth` `x_k^gt`

ATE per axis:

```
ATE_x   = sqrt(mean( (x_k - x_k^gt)² ))
ATE_y   = sqrt(mean( (y_k - y_k^gt)² ))
ATE_pos = sqrt(mean( (x_k - x_k^gt)² + (y_k - y_k^gt)² ))
ATE_yaw = sqrt(mean(angular_wrap(θ_k - θ_k^gt)²))
```

Implementation: subscribe to both `/ekf/odom` and `/ground_truth`, time-
sync, accumulate squared error, take the running RMS.

## 2. Filter consistency — NEES

The Normalised Estimation Error Squared tells us if the filter is
**over-confident** (innovation gate keeps rejecting) or
**under-confident** (covariance huge, no information from updates):

```
NEES_k = (x_k - x_k^gt)^T  Σ_k^{-1}  (x_k - x_k^gt)
```

For a 3-DOF state and 95 % confidence, the expected value is `≈ 3` and
the χ² envelope is `[χ²(3, 0.025), χ²(3, 0.975)] = [0.22, 9.35]`.

- If `NEES > 9.35` consistently → covariance is **too small** (over-
  confident filter → increase `k_r, k_l`).
- If `NEES < 0.22` consistently → covariance is **too large**
  (under-confident → decrease noise gains).
- If it bounces inside the band → well-tuned.

## 3. Detection rate

```
detection_rate = frames_with_marker / total_frames
```

Already published by `aruco_detector` at 1 Hz. Useful to log how often
the EKF can actually correct vs only predict. In the default arena
(robot at centre + 4 markers on walls 2-3 m away) we observe
≈ 100 % at the centre, dropping to ≈ 60 % when the robot's heading
puts all markers behind the camera.

## 4. Occlusion / dropout test

Manually disable detections of a subset of markers (e.g. by covering
them in Gazebo with a temporary box) and observe:

- the covariance ellipse grows (visible in RViz / `/ekf/covariance_ellipse`)
- when the marker reappears, the ellipse shrinks instantly
- the EKF pose snaps back to ground truth on the next update

This is the qualitative demonstration the PDF asks for under
*"confidence ellipsoids and their evolution over time"*.

## 5. Reject rate

```
rejects = #(d² > mahalanobis_gate)
reject_rate = rejects / total_observations
```

Counted in `_correct()` of `ekf_node.py`; published as `reject:...` on
`/ekf/state`. A healthy run keeps this below 5 %; higher means the
detector is producing more outliers than the gate expected.

## How to collect these metrics

Run a session, record a bag:

```bash
./run.sh        # menu -> 1 -> 1 (Part 1 sim)
# in another terminal:
ros2 bag record /ekf/odom /ground_truth /ekf/state /aruco/observations
# drive the robot for ~2 min
# Ctrl+C the bag
```

Then offline:

```python
# scripts/analyse_bag.py — left as a stub for the team
import rosbag2_py, numpy as np
# load /ekf/odom & /ground_truth, compute the metrics above
```

For the video, the **qualitative** demonstration (covariance ellipse
growing/shrinking with marker visibility) is what the PDF emphasises;
the numbers above are the rigorous backup for the report.
