# 🤖 TE3003B: Integration of Robotics and Intelligent Systems (2026)
### 🚀 Final Project & Challenges Repository

![Build](https://img.shields.io/badge/Build-Passing-brightgreen)
![Lint](https://img.shields.io/badge/Lint-Success-blue)
![Status](https://img.shields.io/badge/Status-In%20Progress-yellow)
![Version](https://img.shields.io/badge/Version-1.0.0-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)

<div align="center">
  <img src="./media/logo.png" width="200px" alt="MCR2 Logo">
  <br>
  <i>Developed for the 8th Semester - Robotics Engineering</i>
</div>

---

## 📖 Overview
Repository for the **TE3003B** block at Tec de Monterrey. Implementation of localization, navigation, and path planning for the **Puzzlebot Jetson Edition**.

## 🗓️ Roadmap & Progress
- [x] **[Week 1: Dynamical Systems](./challenges/week1)** (URDF & Puzzlebot Modelling) ✅
- [ ] **Week 2: Mobile Robots Fundamentals** (Kinematics & Point-to-point)
- [ ] **Week 3: Probabilities** (Random variables & Multi-robot plotting)
- [ ] **Week 4: Uncertainty** (Confidence Ellipsoid & Robot Experiments)
- [ ] **Week 5: Reactive Navigation** (Obstacle Avoidance: Bug 0/2)
- [ ] **Week 6: Sources of Information** (Bayes & Kalman Filter 2D)
- [ ] **Week 7: Aruco & Kalman** (Visual Localisation)
- [ ] **Week 8-9: Final Challenge** (Integration & Grading)

---

## 🤝 How to Collaborate (Team Guide)

Follow these steps to keep the repo clean and professional:

### 1. Creating an Issue
Before starting a new task, create an **Issue** on GitHub:
- Explain what you're working on (e.g., "Implement Bug 2 algorithm").
- Assign it to yourself.

### 2. Working with Branches
**Never** commit directly to `main`. Create a feature branch:
```bash
git checkout -b feature/week-2-kinematics
```

### 3. Commit & Push
Use clear commit messages:
```bash
git add .
git commit -m "feat: implement puzzlebot kinematic model"
git push origin feature/week-2-kinematics
```

### 4. Pull Requests (PR)
When the work is ready:
- Open a **PR** to merge your branch into `main`.
- Wait for the **GitHub Actions** to pass (Build & Lint).
- Ask for a review.

---

## 🛠️ Quick Setup
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch puzzlebot_sim puzzlebot_launch.py
```

---
> [!IMPORTANT]
> All proprietary designs belong to Manchester Robotics Ltd. (MCR2).
