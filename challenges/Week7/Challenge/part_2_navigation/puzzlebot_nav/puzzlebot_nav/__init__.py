"""
puzzlebot_nav — Final Challenge · Part 2 · Multi-Waypoint Reactive Navigation

ROS 2 package that extends the Week 6 Bug 0 / Bug 2 stack with a waypoint
manager that chains >=4 goals into a closed trajectory, as required by the
Final Challenge Part 2.

Sub-packages:
    perception/    — LiDAR processing (/scan -> front/left/right sectors)
    navigation/    — Bug 0, Bug 2, and waypoint_manager (closed-loop chaining)
    localisation/  — Dead reckoning + Sigma_k (reused from Mini Challenge 4)
    control/       — PID-to-waypoint controller + goal publisher
"""
