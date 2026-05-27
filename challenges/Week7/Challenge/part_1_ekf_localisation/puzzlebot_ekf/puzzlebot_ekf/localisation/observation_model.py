"""
Observation models for the EKF correction step.

Two models are implemented, as the challenge encourages comparing several:

    (1) range_bearing  — h(x, m) = [r, phi]^T  (2D vector per marker)
        Robust, low-dimensional, well-conditioned even when only one marker
        is visible. This is the model used by the production EKF.

    (2) cartesian      — h(x, m) = [dx, dy]^T  (2D vector per marker)
        Marker position expressed in the robot frame. Cheaper Jacobian
        but yaw is not directly observable from a single point; needs
        >=2 markers to fully constrain pose. Kept for the comparison in
        docs/observation_models.md.

Both models assume the ArUco detector already projects the marker into the
robot frame (XY plane), so the camera-to-robot transform is handled
upstream by aruco_detector.py. This keeps the EKF code independent of
camera intrinsics.
"""

import math
from typing import Tuple

import numpy as np


def h_range_bearing(state: np.ndarray, marker_xy: np.ndarray) -> np.ndarray:
    """Predicted range+bearing of a known marker given the current pose."""
    dx = float(marker_xy[0]) - float(state[0])
    dy = float(marker_xy[1]) - float(state[1])
    r = math.hypot(dx, dy)
    phi = math.atan2(dy, dx) - float(state[2])
    phi = (phi + math.pi) % (2.0 * math.pi) - math.pi
    return np.array([r, phi], dtype=float)


def H_range_bearing(state: np.ndarray, marker_xy: np.ndarray) -> np.ndarray:
    """Jacobian dh/dx (2x3) for the range+bearing model.

    Singular when the robot is exactly on the marker (r=0); the EKF
    skips the correction in that case (handled in ekf_node.py).
    """
    dx = float(marker_xy[0]) - float(state[0])
    dy = float(marker_xy[1]) - float(state[1])
    r2 = dx * dx + dy * dy
    r = math.sqrt(r2)
    if r < 1e-6:
        return np.zeros((2, 3), dtype=float)
    return np.array([
        [-dx / r,    -dy / r,    0.0],
        [ dy / r2,   -dx / r2,  -1.0],
    ], dtype=float)


def innovation_range_bearing(z: np.ndarray,
                             state: np.ndarray,
                             marker_xy: np.ndarray) -> np.ndarray:
    """y = z - h(x), with the bearing component wrapped to (-pi, pi]."""
    z_hat = h_range_bearing(state, marker_xy)
    y = z - z_hat
    y[1] = (y[1] + math.pi) % (2.0 * math.pi) - math.pi
    return y


def h_cartesian(state: np.ndarray, marker_xy: np.ndarray) -> np.ndarray:
    """Predicted (dx, dy) of the marker in the robot frame."""
    dxw = float(marker_xy[0]) - float(state[0])
    dyw = float(marker_xy[1]) - float(state[1])
    c = math.cos(float(state[2]))
    s = math.sin(float(state[2]))
    return np.array([ c * dxw + s * dyw,
                     -s * dxw + c * dyw], dtype=float)


def H_cartesian(state: np.ndarray, marker_xy: np.ndarray) -> np.ndarray:
    """Jacobian dh/dx (2x3) for the cartesian model."""
    dxw = float(marker_xy[0]) - float(state[0])
    dyw = float(marker_xy[1]) - float(state[1])
    c = math.cos(float(state[2]))
    s = math.sin(float(state[2]))
    return np.array([
        [-c, -s, -s * dxw + c * dyw],
        [ s, -c, -c * dxw - s * dyw],
    ], dtype=float)


def innovation_cartesian(z: np.ndarray,
                         state: np.ndarray,
                         marker_xy: np.ndarray) -> np.ndarray:
    return z - h_cartesian(state, marker_xy)


def get_model(name: str) -> Tuple[callable, callable, callable]:
    """Factory used by ekf_node.py to swap models via parameter."""
    name = name.lower()
    if name in ('range_bearing', 'rb'):
        return h_range_bearing, H_range_bearing, innovation_range_bearing
    if name in ('cartesian', 'xy'):
        return h_cartesian, H_cartesian, innovation_cartesian
    raise ValueError(
        f"Unknown observation model '{name}'. "
        f"Use 'range_bearing' or 'cartesian'."
    )
