"""
Differential-drive motion model used by the EKF prediction step.

State:     x = [x, y, theta]^T  (robot pose in the world frame)
Control:   u = [omega_r, omega_l]^T  (wheel angular velocities)

The model is the same as the dead-reckoning integrator from Mini Challenge 4
(Week 5), repackaged here as a pure function plus its Jacobians so the EKF
can call it cleanly. Keeping it pure (no ROS, no class state) makes it
trivially unit-testable and reusable for both Part 1 and the optional
integration in Part 3.
"""

import math
from typing import Tuple

import numpy as np


def predict_state(x: np.ndarray,
                  u: np.ndarray,
                  dt: float,
                  wheel_radius: float,
                  wheelbase: float) -> Tuple[np.ndarray, float, float]:
    """Euler-integrate the differential-drive kinematics one step forward.

    Returns the predicted state and the (v, w) actually applied so the
    caller can reuse them for the Jacobians without recomputing.
    """
    omega_r, omega_l = float(u[0]), float(u[1])
    v = wheel_radius * (omega_r + omega_l) / 2.0
    w = wheel_radius * (omega_r - omega_l) / wheelbase

    theta_prev = float(x[2])
    x_new = np.array([
        float(x[0]) + v * math.cos(theta_prev) * dt,
        float(x[1]) + v * math.sin(theta_prev) * dt,
        theta_prev + w * dt,
    ], dtype=float)
    return x_new, v, w


def motion_jacobian_state(theta_prev: float,
                          v: float,
                          dt: float) -> np.ndarray:
    """Jacobian F = df/dx evaluated at the *previous* yaw.

    Same H used in the Week 5 dead-reckoning covariance propagation. Note
    that F is evaluated at theta_{k-1}, not at theta_k.
    """
    s = math.sin(theta_prev)
    c = math.cos(theta_prev)
    return np.array([
        [1.0, 0.0, -dt * v * s],
        [0.0, 1.0,  dt * v * c],
        [0.0, 0.0,  1.0       ],
    ], dtype=float)


def motion_jacobian_control(theta_prev: float,
                            dt: float,
                            wheel_radius: float,
                            wheelbase: float) -> np.ndarray:
    """Jacobian W = df/du (3x2), evaluated at theta_{k-1}.

    Used to map wheel-input noise into pose-space process noise:
        Q = W * Sigma_delta * W^T
    """
    s = math.sin(theta_prev)
    c = math.cos(theta_prev)
    r = wheel_radius
    L = wheelbase
    return 0.5 * r * dt * np.array([
        [c,        c       ],
        [s,        s       ],
        [2.0 / L, -2.0 / L ],
    ], dtype=float)


def process_noise_input(omega_r: float,
                        omega_l: float,
                        k_r: float,
                        k_l: float) -> np.ndarray:
    """Input-space covariance Sigma_delta (2x2).

    Noise grows with the magnitude of each wheel velocity, as in Week 5.
    A stationary wheel injects no noise, which keeps the filter quiet
    when the robot is parked.
    """
    return np.diag([k_r * abs(omega_r), k_l * abs(omega_l)])


def process_noise_state(theta_prev: float,
                        omega_r: float,
                        omega_l: float,
                        dt: float,
                        wheel_radius: float,
                        wheelbase: float,
                        k_r: float,
                        k_l: float) -> np.ndarray:
    """Process noise Q in state space (3x3), Q = W * Sigma_delta * W^T."""
    W = motion_jacobian_control(theta_prev, dt, wheel_radius, wheelbase)
    Sd = process_noise_input(omega_r, omega_l, k_r, k_l)
    return W @ Sd @ W.T


def wrap_to_pi(angle: float) -> float:
    """Map any angle to (-pi, pi]. Essential after every yaw update."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
