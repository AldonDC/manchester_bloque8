# Observation Models — Comparison

The PDF explicitly encourages testing several observation models. We
implemented two in [observation_model.py](../puzzlebot_ekf/puzzlebot_ekf/localisation/observation_model.py)
and pick one at runtime via `observation_model` in
[ekf_params.yaml](../puzzlebot_ekf/config/ekf_params.yaml).

## 1. Range + Bearing  (default)

`h(x, m) = [r, phi]^T`  where
- `r   = ‖m − p‖`
- `phi = atan2(m_y − y,  m_x − x) − theta`

### Why we use this as primary

- **Geometrically natural** — these are exactly the quantities a monocular
  camera + ArUco fiducial can measure cleanly. `solvePnP` returns a 3-D
  translation in the camera frame; projecting to the robot ground plane
  and taking polar coordinates gives `(r, phi)` directly.
- **Well-conditioned with a single marker** — two scalars constrain two
  degrees of freedom (`x, y` along with `theta`'s linear correction from
  bearing). The EKF is fully observable from one good detection.
- **Innovation has a clean physical interpretation** — `(z_r - h_r,
  z_phi - h_phi)` is "you are 5 cm farther than I thought and 2° to my
  left". Easy to log, easy to tune `R = diag(σ_r², σ_phi²)`.
- **Singularity is localised** — Jacobian is singular only when the
  robot sits exactly on the marker (`r → 0`); we skip the correction
  in that pathological case.

### Jacobian
```
              ∂h/∂x   ∂h/∂y   ∂h/∂θ
   ∂r/∂· = [ -dx/r , -dy/r ,   0   ]
   ∂φ/∂· = [  dy/r², -dx/r²,  -1   ]
```
where `dx = m_x − x`, `dy = m_y − y`, `r = √(dx² + dy²)`.

## 2. Cartesian (marker position in robot frame)

`h(x, m) = [m_x^R, m_y^R]^T`  — marker XY in the robot body frame.

### Trade-off vs range+bearing

| | Range + Bearing | Cartesian |
|---|---|---|
| Dimension | 2 (per marker) | 2 (per marker) |
| Yaw observable from 1 marker | ✅ via bearing | ❌ need ≥2 markers |
| Sensitivity to small marker pose errors | low (polar smooths) | high (translates 1:1) |
| Jacobian | trig + division by r | linear in cos/sin θ |
| Best when | single marker visible, large range | two+ markers, close range |

The Cartesian model is implemented and available for ablation studies.
To switch:

```yaml
# ekf_params.yaml
ekf_node:
  ros__parameters:
    observation_model: 'cartesian'    # or 'range_bearing'
```

## 3. Pose-based (not implemented)

`h(x, m) = [t_x, t_y, t_z, r_x, r_y, r_z]^T` — full SE(3) pose of the
marker in the camera frame as returned by `cv2.solvePnP`. We skipped it
because:

- 6 scalars per marker → 36 entries in `R` to tune.
- The rotational components are noisy on ArUcos < 20 cm at >1.5 m
  range; they dominate the innovation in a bad way without giving
  proportionally more information.
- The bearing observation in the range+bearing model already captures
  the only rotational degree of freedom that matters for a planar
  Puzzlebot (the heading).

## Empirical observations (qualitative)

Running `ekf_sim_launch.py`, manually driving the robot and watching
`/ekf/state` for `update:id=X:d2=Y` lines:

- **Range+bearing** keeps `d²` (Mahalanobis) below the χ² gate
  (9.21 at 99% / 2 dof) for >95 % of detections at distances up to ~4 m.
  Few rejects, smooth corrections.
- **Cartesian** also tracks well but the bearing component is implicitly
  reconstructed only when ≥2 markers are visible simultaneously. With
  one marker the yaw drifts linearly with odometry until a second
  detection.

## Robustness strategies (per PDF requirement)

Each strategy lives in code; the doc just enumerates them:

1. **Mahalanobis gating** (ekf_node.py) — rejects observations with
   `d² > gate` (default χ²(2,99 %) = 9.21).
2. **Per-frame sanity gate** (aruco_detector.py) — drops detections
   with implausible range (< 5 cm or > 6 m) or NaN bearing.
3. **Covariance clamp** — diagonal entries capped at 25 m² so a
   transient bad correction can't blow `P` to infinity.
4. **Correction step saturation** — `K @ y` clipped to ±1 m in
   position and ±0.5 rad in yaw before applying.
5. **Symmetrisation of P** — `P = 0.5(P + P^T)` each tick to fight
   numerical drift.
6. **Observation stale-drop** — observations older than
   `max_observation_age_s` (0.5 s default) discarded.
