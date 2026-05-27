"""
puzzlebot_ekf — Final Challenge · Part 1 · Camera-based EKF Localisation

ROS 2 package implementing Extended Kalman Filter localisation for the
Puzzlebot, fusing wheel odometry (prediction) with ArUco marker observations
from a monocular camera (correction).

Sub-packages:
    perception/    — ArUco detection + camera calibration loader
    localisation/  — EKF node, motion/observation models, covariance ellipsoids
    map/           — known ArUco marker map (id -> pose in world)
    utils/         — coordinate transforms (robot / camera / marker frames)

Allowed libraries (per challenge rules):
    ArUco library (OpenCV contrib), OpenCV, NumPy. Nothing else.
"""
