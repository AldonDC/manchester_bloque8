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

### [Week 1: Dynamical Systems](./challenges/week1) ✅

Basics of dynamical systems and modern autonomous systems.

* **Session**: Transforms in ROS2, URDF, Markers.
* **Activity**:  Modelling in ROS 2.
* **Mini Challenge**: [Puzzlebot URDF Modelling &amp; RVIZ Visualization](./challenges/week1/puzzlebot_sim).

### Week 2: Mobile Robots – Fundamentals

Review of mobile robotics kinematics and navigation.

* **Session**: Kinematics for differential drive, Dead Reckoning (Encoder-based).
* **Mini Challenge**: Kinematic model of Puzzlebot & Point-to-point navigation.

### Week 3: Probabilities in Robotics

Introduction to probabilities and linearization.

* **Session**: Random variables (Discrete/Continuous), Distributions (Uniform/Gaussian).
* **Mini Challenge**: Multi-robot plotting in ROS 2.

### Week 4: Uncertainty in Mobile Robotics

Localized navigation in the presence of uncertainties.

* **Session**: Ellipsoid of confidence, Mobile robot localisation (dead reckoning).
* **Mini Challenge**: Open-loop experiments (Straight/Turn), Confidence ellipsoids.

### Week 5: Reactive Navigation

Introduction to obstacle avoidance algorithms.

* **Session**: Exteroceptive sensors, Bug 0, Bug 1, Bug 2.
* **Mini Challenge**: Implementation of Bug 0 and Bug 2 (Gazebo & Real Robot).

### Week 6: Sources of Information

Introduction to the Kalman Filter.

* **Session**: Bayes Filter, Kalman Filter for map-based localisation (2D).
* **Mini Challenge**: Map-based localisation experiments.

### Week 7: Visual Localisation & Final Challenge

Integration of computer vision and filtering.

* **Session**: Camera-based localisation, Aruco markers, 3D Kalman Filter.
* **Final Challenge**: Integrated navigation with obstacles using Kalman estimation.

### Week 8-9: Final Challenge & Grading

Q&A sessions and final project delivery.

* **Goal**: Full system integration and grading.

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
