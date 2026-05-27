# Part 2 — Closed Multi-Waypoint Reactive Navigation

Extends the Week 6 Mini Challenge (Bug 0 / Bug 2) with a **waypoint
manager** that chains four or more goals into a closed trajectory, as
required by the Final Challenge.

## What's new vs. Week 6

| | Week 6 (`puzzlebot_bug_w6`) | Week 7 Part 2 (`puzzlebot_nav`) |
|---|---|---|
| Goal source | Single `goal_publisher` (one fixed (x_T, y_T)) | `waypoint_manager` sequences ≥4 waypoints |
| Trajectory | One-shot from `(0,0)` to `(x_T, y_T)` | Closed loop p₀ → p₁ → … → p₀ |
| Termination | `GOAL_REACHED` and stop | Reach each wp, dwell briefly, advance, loop |
| Config | Hard-coded arg | YAML file ([config/waypoints.yaml](puzzlebot_nav/config/waypoints.yaml)) |

Everything else (`lidar_processor`, `localisation`, `controller`, `bug0`,
`bug2`, the worlds, the RViz config) is reused verbatim from Week 6.

## Architecture

```
/scan ── lidar_processor ──► /bug/d_front, /bug/d_left, /bug/d_right ──┐
/wr, /wl ── localisation ──► /odom                                       │
                                                                         ├─► bug2 ──► /set_point ──► controller ──► /cmd_vel
config/waypoints.yaml ──► waypoint_manager ──► /goal ─────────────────► ┤
                                  ▲                                      │
                                  └───── /bug/state ◄─────────────────── ┘
```

The `waypoint_manager` keeps `bug0`/`bug2` single-goal — they don't even
know about the closed loop. The manager publishes the current goal,
watches `/bug/state` for `GOAL_REACHED`, dwells `dwell_time` seconds, then
advances. After the last waypoint it cycles back to `p₀` (closed loop).

## Waypoint design

The PDF requires:

* At least 4 target points
* No straight paths between consecutive points
* No two waypoints in the same direction
* The robot must circumnavigate all obstacles

The default `waypoints.yaml` was designed for `bug_medium.world`:

```yaml
waypoints:
  - [2.0, 0.0]
  - [2.0, 2.0]
  - [0.0, 2.0]
  - [-1.5, 1.0]
```

Edit the YAML — no rebuild needed (it's a runtime config file).

## Running

```bash
# Default: bug2 on bug_medium.world
ros2 launch puzzlebot_nav nav_multipoint_launch.py

# Switch to bug0
ros2 launch puzzlebot_nav nav_multipoint_launch.py bug:=bug0

# Try a harder world
ros2 launch puzzlebot_nav nav_multipoint_launch.py world:=bug_hard.world
```

You can also still run the original Week 6 single-goal launches inside
this package (`bug0_launch.py`, `bug2_launch.py`) for sanity checks.

## Diagnostics

* `ros2 topic echo /waypoint_manager/state` — RUNNING / DWELL / FINISHED + current wp index
* `ros2 topic echo /bug/state` — Bug FSM state (GO_TO_GOAL / FOLLOW_WALL / GOAL_REACHED)
* RViz: blue spheres = remaining waypoints, orange = current target, blue line = planned loop
