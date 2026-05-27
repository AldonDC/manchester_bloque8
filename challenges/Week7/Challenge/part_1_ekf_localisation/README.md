# Part 1 — Camera-based EKF Localisation

Estimates the Puzzlebot's pose by fusing:

* **Prediction** — wheel-odometry motion model from Week 5
  (differential-drive Euler integration + Jacobian-based covariance propagation).
* **Correction** — ArUco marker observations from a monocular camera,
  expressed as range + bearing in the robot frame.

The output is a full pose with covariance `Sigma_k` that gets published as
`nav_msgs/Odometry`, plus a `MarkerArray` of confidence ellipses for RViz.

## Architecture

```
/camera/image_raw  ──┐
/camera/camera_info ─┤── aruco_detector ── /aruco/observations ──┐
                                                                  ├── ekf_node ── /ekf/odom ──┬── RViz
/wr ─────────────────────────────────────────────────────────────┘                            │
/wl ─────────────────────────────────────────────────────────────                              │
                                                                                              │
                                            covariance_publisher ── /ekf/covariance_ellipse ──┘
```

Three nodes, decoupled by topics:

| Node | Source | Responsibility |
|------|--------|----------------|
| `aruco_detector` | [puzzlebot_ekf/perception/aruco_detector.py](puzzlebot_ekf/puzzlebot_ekf/perception/aruco_detector.py) | Detect markers, run `solvePnP`, convert to robot-frame range+bearing |
| `ekf_node`       | [puzzlebot_ekf/localisation/ekf_node.py](puzzlebot_ekf/puzzlebot_ekf/localisation/ekf_node.py) | Predict-correct cycle; outputs `/ekf/odom` |
| `covariance_publisher` | [puzzlebot_ekf/localisation/covariance_publisher.py](puzzlebot_ekf/puzzlebot_ekf/localisation/covariance_publisher.py) | Decompose 2x2 XY covariance into a flat ellipse Marker |

Pure-Python math (no ROS) sits in:

* [motion_model.py](puzzlebot_ekf/puzzlebot_ekf/localisation/motion_model.py)
* [observation_model.py](puzzlebot_ekf/puzzlebot_ekf/localisation/observation_model.py)
* [aruco_map.py](puzzlebot_ekf/puzzlebot_ekf/map/aruco_map.py)

This split keeps the math unit-testable and makes swapping observation
models a one-line change in `ekf_params.yaml`.

## Configuration

All knobs live in [config/ekf_params.yaml](puzzlebot_ekf/config/ekf_params.yaml)
and the marker map in [config/aruco_map.yaml](puzzlebot_ekf/config/aruco_map.yaml).
Key parameters:

| Parameter | What it does | Tune up when |
|-----------|--------------|--------------|
| `k_r`, `k_l` | Per-wheel input noise gain | Filter is over-confident (ellipse too small) |
| `sigma_range`, `sigma_bearing` | Measurement noise per ArUco | ArUco poses are jumpy / shaky |
| `mahalanobis_gate` | Outlier rejection threshold (chi²) | Spurious detections are corrupting belief |
| `observation_model` | `range_bearing` or `cartesian` | See [docs/observation_models.md](docs/observation_models.md) |

## Camera calibration

* **In Gazebo**: leave `camera_intrinsics_yaml` empty. The detector seeds
  K and D from `/camera/camera_info` automatically.
* **On the real robot**: capture a chessboard with
  [calibration/saveImages.py](puzzlebot_ekf/calibration/saveImages.py), then run
  [calibration/CameraCal.py](puzzlebot_ekf/calibration/CameraCal.py) to produce
  `calibration_matrix.yaml`. Point `camera_intrinsics_yaml` to it.

## Running

```bash
# Standalone (expects a bag with /camera/image_raw, /camera/camera_info, /wr, /wl)
ros2 launch puzzlebot_ekf ekf_only_launch.py

# Sim (TODO: ekf_sim_launch.py — Gazebo world with markers, in progress)
ros2 launch puzzlebot_ekf ekf_sim_launch.py
```

## Robustness checklist (from the PDF)

* [x] Multiple markers seen simultaneously — each contributes a correction
* [x] No marker visible — only `predict` runs; covariance grows
* [x] Partial / occluded marker — Mahalanobis gate drops bad detections
* [x] Confidence ellipse rendered in RViz, with optional time trail
* [ ] Quantitative metrics (RMSE, ATE) vs `/ground_truth` — see
      [docs/metrics.md](docs/metrics.md)
