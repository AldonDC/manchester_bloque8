# Final Challenge — Video Script (3–4 min, English)

Target length: **3:30** so we have buffer for editing. PDF requirements
checked off as we go.

## 0:00 — 0:20 · Intro (20 s)

> "Hi, I'm Alfonso Diaz from Tec de Monterrey. For the MCR2 Final
> Challenge I built a camera-based Extended Kalman Filter that fuses
> wheel odometry with ArUco fiducial detections to localise the
> Puzzlebot in an unknown environment, and a multi-waypoint reactive
> navigation that closes a trajectory of four target points around
> obstacles."

Visual: title card → quick clip of the EKF arena in RViz with robot
moving and the covariance ellipse breathing.

## 0:20 — 1:00 · What is a Kalman Filter, in my own words (40 s)

> "An EKF is a two-step probabilistic estimator. **Predict** propagates
> the belief forward using a motion model — for the Puzzlebot, Euler
> integration of differential-drive kinematics — and grows the
> uncertainty through a Jacobian. **Correct** shrinks that uncertainty
> when a measurement arrives; the Kalman gain weights how much we
> trust the measurement versus the prediction.
>
> 'Extended' just means we linearise non-linear models around the
> current mean — necessary because both the motion model and the
> camera observation model are non-linear."

Visual: animation of the predict-correct loop, equations on screen
- `x_k|k-1 = f(x_{k-1}, u_k)`
- `P_k|k-1 = F P F^T + Q`
- `y = z - h(x_k|k-1)`
- `S = H P H^T + R`
- `K = P H^T S^-1`
- `x_k = x_k|k-1 + K y`
- `P_k = (I - K H) P_k|k-1`

## 1:00 — 1:50 · How we implemented it (50 s)

> "Three ROS 2 nodes, decoupled by topics.
>
> [`aruco_detector`](part_1_ekf_localisation/puzzlebot_ekf/puzzlebot_ekf/perception/aruco_detector.py)
> subscribes to `/camera/image_raw`, runs OpenCV's ArUco detection,
> calls `solvePnP` for the marker pose in the camera frame, and
> publishes `(id, range, bearing)` per visible marker as a `PoseArray`.
>
> [`ekf_node`](part_1_ekf_localisation/puzzlebot_ekf/puzzlebot_ekf/localisation/ekf_node.py)
> runs the predict step at 50 Hz from `/wr` and `/wl` wheel velocities,
> and runs the correct step every time a marker observation arrives.
> The observation model is range+bearing — I picked it because two
> scalars are enough to fully constrain the 3-DOF pose when even one
> marker is visible.
>
> [`covariance_publisher`](part_1_ekf_localisation/puzzlebot_ekf/puzzlebot_ekf/localisation/covariance_publisher.py)
> decomposes the 2×2 XY block of the pose covariance, builds a flat
> ellipse marker and publishes it for RViz with a time trail so we
> see the uncertainty evolution."

Visual: rqt_graph / workflow diagram from the PDF, then code peeks at
each node's `__init__` and the predict/correct functions.

## 1:50 — 2:30 · Robustness strategies (40 s)

> "Five strategies against bad observations:
> 1. **Mahalanobis gating** — any observation whose innovation
>    distance is above χ²(2, 99 %) = 9.21 is rejected outright.
> 2. **Per-frame sanity gate** in the detector — observations with
>    range below 5 cm or above 6 m, or non-finite bearing, are dropped
>    before they reach the EKF.
> 3. **Covariance clamping** — diagonal entries capped at 25 m² so a
>    single bad update can't blow the belief to infinity.
> 4. **Correction step saturation** — `K · y` clipped to ±1 m and
>    ±30°, again so one bad observation can't teleport the robot.
> 5. **Symmetrisation of P** every tick — fights numerical drift that
>    would otherwise make `P` non-positive-definite."

Visual: cover one of the four ArUcos in Gazebo manually → covariance
ellipse grows → uncover → ellipse snaps back. This is exactly the
demonstration the PDF asks for.

## 2:30 — 3:00 · Part 2 — multi-waypoint navigation (30 s)

> "For Part 2 I extended Week 6's Bug 0 / Bug 2 reactive navigation
> with a `waypoint_manager` node. It loads four waypoints from a YAML
> file, publishes them one at a time to `/goal`, watches `/bug/state`
> for `GOAL_REACHED`, dwells one second, advances to the next, and
> after the last one loops back to the first to close the trajectory."

Visual: RViz top-down view of `bug_medium.world` showing the four
blue spheres + line strip + the robot following bug2 around the
obstacles.

## 3:00 — 3:30 · Challenges & wrap-up (30 s)

> "The biggest engineering challenge was *not* the math — that was
> well-defined. It was the integration: Gazebo Harmonic and Ogre2 have
> several legacy-material edge cases that drop visuals silently, and
> the EKF has to stay numerically clean for tens of thousands of
> ticks across observations dropping in and out. The combination of
> Mahalanobis gating + covariance clamping fixed all the divergence
> cases I saw in testing.
>
> Future work: combining Parts 1 and 2 (Part 3 in the PDF) into a
> single supervisor where the EKF feeds the waypoint navigation so
> the robot uses its filtered belief instead of dead-reckoning.
>
> Thanks for watching. Code at GitHub link in the description."

Visual: side-by-side of the EKF arena (Part 1) and the Bug 2
multi-waypoint demo (Part 2), end with the credits card.

---

## Recording checklist

- [ ] RViz window size 1920×1080
- [ ] Hide left/right RViz dock panels (cleaner shot)
- [ ] Enable Ground Truth display in RViz (toggle in launch)
- [ ] OBS Studio: record at 30 fps, capture only the RViz window
- [ ] Voiceover separately, mix in post (or use whatever your campus accepts)
- [ ] Upload to YouTube unlisted, share link in the submission form

## Submission deadlines (Central Mexico Time)

- Video submission: **2 June 2026, 4 PM**
- Final presentation:  **3 June 2026, 1 PM**
