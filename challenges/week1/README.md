# 🧩 Week 1 Challenge: Dynamical Systems & Modelling

## 📋 Objective
The goal of this week is to understand the basics of dynamical systems and learn how to model robots in ROS 2 using **URDF** (Unified Robot Description Format) and visualize them in **RViz**.

## 🎯 Tasks
1.  **Drone Modelling**: Understand transforms and markers by modelling a drone.
2.  **Puzzlebot Modelling**: Create a complete URDF model of the **Puzzlebot Jetson/Lidar Edition**.
3.  **Visualization**: Use `joint_state_publisher` and `robot_state_publisher` to visualize the robot in RViz with its simplified meshes.

## 📂 Deliverables
- [ ] **URDF File**: `urdf/puzzlebot.urdf`
- [ ] **Launch File**: `launch/puzzlebot_launch.py`
- [ ] **Python Scripts**: `puzzlebot_sim/joint_state_publisher.py` (Custom implementation)

## 🛠️ Instructions
### 1. Build the package
Ensure you are in the root of the workspace:
```bash
colcon build --packages-select puzzlebot_sim
source install/setup.bash
```

### 2. Launch the Simulation
To visualize the current state of the Puzzlebot:
```bash
ros2 launch puzzlebot_sim puzzlebot_launch.py
```

## 📝 Notes
- Check the official PDF for detailed dimensions: [MCR2_Puzzlebot_Modelling_challenge.pdf](./MCR2_Puzzlebot_Modelling_challenge.pdf)
- Use the provide meshes in the `meshes/` directory for a realistic look.
