# Week 7 — Final Challenge: Autonomous Exploration

Manchester Robotics × NVIDIA · TE3003B

This is the Final Challenge of the course. The Puzzlebot must autonomously
explore an unknown environment in Gazebo (and optionally on the real robot).
The challenge is split into three parts:

| Part | Topic | Status |
|------|-------|--------|
| [Part 1](part_1_ekf_localisation/) | Camera-based EKF localisation with ArUco markers and wheel odometry | **Required** |
| [Part 2](part_2_navigation/)        | Closed-loop multi-waypoint reactive navigation (Bug 0 / Bug 2)       | **Required** |
| [Part 3](part_optional_integration/) | Integration of Parts 1 + 2 — EKF-localised navigation               | Optional     |

Each part is a self-contained ROS 2 package so they can be developed,
tested, and graded independently.

## Quick start

```bash
# Build everything
cd ~/ros2_ws            # or wherever your workspace lives
colcon build --packages-select puzzlebot_ekf puzzlebot_nav
source install/setup.bash

# Part 1 — EKF only (needs a camera stream + /wr, /wl)
ros2 launch puzzlebot_ekf ekf_only_launch.py

# Part 2 — Multi-waypoint navigation in Gazebo
ros2 launch puzzlebot_nav nav_multipoint_launch.py world:=bug_medium.world
```

For end-to-end shortcuts, use the project launcher:

```bash
./run.sh part1     # EKF stack (no Gazebo, expects bag or real cam)
./run.sh part2     # Multi-waypoint nav in Gazebo
./run.sh full      # Integrated EKF + navigation (Part 3, when ready)
```

## Library policy

The challenge rules forbid external libraries beyond:

* **Part 1**: ArUco library, OpenCV, NumPy / Eigen / LAPACK.
* **Part 2**: no external libraries at all (only ROS 2 client + NumPy).

The packages in this repository respect those limits — no SciPy, no
robot_localization, no nav2.

## Deliverables

* 3–4 min YouTube video (English) explaining the EKF, the navigation, and
  the comparisons.
* Q&A: 2 min with MCR2, then 3 min with the judges if selected as finalist.
* **Deadlines** (Central Mexico time):
  * Video submission — 2 June 2026, 4 PM
  * Final presentation — 3 June 2026, 1 PM

## Source

Built on top of the Week 6 reactive navigation Mini Challenge
(`puzzlebot_bug_w6`, kept untouched at [../../Week6/Challenge](../../Week6/Challenge/))
and the Week 7 lab activities (ArUco detection, camera calibration).
