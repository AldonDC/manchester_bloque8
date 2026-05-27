"""
ekf_node — Extended Kalman Filter localisation node (Final Challenge Part 1).

Predicts the Puzzlebot pose with wheel-odometry (Week 5 motion model) and
corrects it whenever an ArUco marker observation arrives. The corrected
pose, with its covariance Sigma_k, is published as nav_msgs/Odometry so
RViz draws the covariance ellipse automatically; a sister node publishes
a MarkerArray ellipsoid for nicer rendering and tracking over time.

Contract
--------
Subscribes:
    /wr, /wl                              std_msgs/Float32  wheel velocities
    /aruco/observations                   PoseArray*        per-marker (r, phi)
        * PoseArray is repurposed because the challenge bans extra ROS
          message packages; each Pose.position.x = marker_id, .y = range,
          .z = bearing. Decoded in _aruco_cb. Simple and self-contained.

Publishes:
    /ekf/odom                             nav_msgs/Odometry pose + covariance
    /ekf/state                            std_msgs/String   debug

Parameters mirror Week 5 plus EKF-specific noise terms; see config/ekf_params.yaml.
"""

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from puzzlebot_ekf.localisation.motion_model import (
    predict_state, motion_jacobian_state, process_noise_state, wrap_to_pi,
)
from puzzlebot_ekf.localisation.observation_model import get_model
from puzzlebot_ekf.map.aruco_map import ArucoMap


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        # ── Physical and timing parameters (Week 5 conventions) ───────────────
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheelbase', 0.19)
        self.declare_parameter('sample_time', 0.02)
        self.declare_parameter('x_init', 0.0)
        self.declare_parameter('y_init', 0.0)
        self.declare_parameter('theta_init', 0.0)

        # ── Process noise (Week 5 wheel-input model) ──────────────────────────
        self.declare_parameter('k_r', 0.05)
        self.declare_parameter('k_l', 0.05)

        # ── Measurement noise R (per observation) ─────────────────────────────
        # range_bearing: diag(sigma_r^2, sigma_phi^2)
        # cartesian:     diag(sigma_x^2, sigma_y^2)
        self.declare_parameter('sigma_range', 0.05)
        self.declare_parameter('sigma_bearing', 0.05)
        self.declare_parameter('sigma_xy', 0.05)

        # ── Robustness gates ──────────────────────────────────────────────────
        self.declare_parameter('observation_model', 'range_bearing')
        self.declare_parameter('mahalanobis_gate', 9.21)  # chi2(2 dof, 99 %)
        self.declare_parameter('max_observation_age_s', 0.5)
        self.declare_parameter('aruco_map_yaml', '')

        # ── Cache ─────────────────────────────────────────────────────────────
        gp = lambda n: self.get_parameter(n).value
        self.r = float(gp('wheel_radius'))
        self.l = float(gp('wheelbase'))
        self.dt = float(gp('sample_time'))
        self.k_r = float(gp('k_r'))
        self.k_l = float(gp('k_l'))
        self.gate = float(gp('mahalanobis_gate'))
        self.max_age = float(gp('max_observation_age_s'))

        model_name = self.get_parameter('observation_model').get_parameter_value().string_value
        self._h, self._H, self._innov = get_model(model_name)
        self.model_name = model_name

        if model_name in ('range_bearing', 'rb'):
            self.R = np.diag([
                float(gp('sigma_range')) ** 2,
                float(gp('sigma_bearing')) ** 2,
            ])
        else:
            sxy = float(gp('sigma_xy'))
            self.R = np.diag([sxy ** 2, sxy ** 2])

        # ── Marker map ────────────────────────────────────────────────────────
        map_path = self.get_parameter('aruco_map_yaml').get_parameter_value().string_value
        if not map_path:
            raise RuntimeError(
                "Parameter 'aruco_map_yaml' is empty. "
                "Pass the absolute path to your aruco_map.yaml."
            )
        self.aruco_map = ArucoMap.from_yaml(map_path)
        self.get_logger().info(f'Loaded {self.aruco_map}')

        # ── Initial belief ────────────────────────────────────────────────────
        self.x = np.array([float(gp('x_init')),
                           float(gp('y_init')),
                           float(gp('theta_init'))], dtype=float)
        self.P = np.zeros((3, 3), dtype=float)

        # ── Last wheel velocities (control input) ─────────────────────────────
        self.wr = 0.0
        self.wl = 0.0

        # ── Evaluation metrics state ──────────────────────────────────────────
        # Para el PDF Part 1: error vs ground truth, trace(P), conteos de
        # updates/rejects por marker. Se imprime cada 1s en _metrics_tick().
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._gt = None                          # último ground_truth
        self._updates = {}                       # marker_id -> count aceptados
        self._rejects = {}                       # marker_id -> count rechazados
        self._last_obs_ids = []                  # ids vistos en el último _aruco_cb

        # ── ROS plumbing ──────────────────────────────────────────────────────
        self.create_subscription(Float32, 'wr', self._wr_cb, 10)
        self.create_subscription(Float32, 'wl', self._wl_cb, 10)
        self.create_subscription(PoseArray, 'aruco/observations',
                                 self._aruco_cb, 10)
        # Ground truth para calcular el error de localización (PDF Part 1:
        # "métricas de evaluación de su algoritmo para verificar robustez").
        self.create_subscription(Odometry, 'ground_truth',
                                 self._gt_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, 'ekf/odom', 10)
        self.pub_state = self.create_publisher(String, 'ekf/state', 10)
        self.pub_map = self.create_publisher(MarkerArray, 'ekf/aruco_map', 10)
        # Broadcast odom -> base_footprint so RViz can render the RobotModel
        # without depending on robot_state_publisher's link_joint TFs lining
        # up with our localisation belief.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self._tick)
        # Republish the known marker map once a second so RViz picks it up
        # even after a late join. Cheap (small array).
        self.map_timer = self.create_timer(1.0, self._publish_map_markers)
        # Métricas cada 1 s en la terminal.
        self.metrics_timer = self.create_timer(1.0, self._metrics_tick)

        self.get_logger().info(
            f'EKF up | model={self.model_name} | dt={self.dt}s | '
            f'k_r={self.k_r} k_l={self.k_l} | gate={self.gate:.2f}'
        )

    # ───────────────────────────────────────────────────────────────────────
    # Subscription callbacks
    # ───────────────────────────────────────────────────────────────────────

    def _wr_cb(self, msg: Float32):
        self.wr = float(msg.data)

    def _wl_cb(self, msg: Float32):
        self.wl = float(msg.data)

    def _aruco_cb(self, msg: PoseArray):
        """Apply EKF correction for each fresh, mapped observation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0 and (now - stamp) > self.max_age:
            return  # stale, drop it

        ids_now = []
        for pose in msg.poses:
            marker_id = int(round(pose.position.x))
            z0 = float(pose.position.y)
            z1 = float(pose.position.z)
            ids_now.append(marker_id)
            self._correct(marker_id, np.array([z0, z1], dtype=float))
        self._last_obs_ids = ids_now

    def _gt_cb(self, msg: Odometry):
        """Cache ground-truth pose for periodic error metric."""
        q = msg.pose.pose.orientation
        # yaw from (z, w) — la odometría del simulador es plana.
        yaw = 2.0 * math.atan2(q.z, q.w)
        self._gt = (msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw)

    # ───────────────────────────────────────────────────────────────────────
    # EKF steps
    # ───────────────────────────────────────────────────────────────────────

    def _tick(self):
        """One predict cycle. Corrections are event-driven in _aruco_cb."""
        u = np.array([self.wr, self.wl], dtype=float)
        theta_prev = float(self.x[2])

        x_new, v, _w = predict_state(
            self.x, u, self.dt, self.r, self.l)
        F = motion_jacobian_state(theta_prev, v, self.dt)
        Q = process_noise_state(theta_prev, self.wr, self.wl, self.dt,
                                self.r, self.l, self.k_r, self.k_l)

        self.x = x_new
        self.x[2] = wrap_to_pi(self.x[2])
        self.P = F @ self.P @ F.T + Q

        # Numerical hygiene: enforce symmetry and clamp variance so a transient
        # bad correction can't make the belief blow up (we saw the RViz arrow
        # shoot to infinity with magenta "rays" when P grew unbounded).
        self.P = 0.5 * (self.P + self.P.T)
        np.fill_diagonal(self.P, np.minimum(np.diag(self.P), 25.0))

        self._publish_odom(v, _w)

    def _correct(self, marker_id: int, z: np.ndarray):
        marker_xy = self.aruco_map.xy(marker_id)
        if marker_xy is None:
            self.pub_state.publish(String(data=f'drop:unmapped:{marker_id}'))
            return

        H = self._H(self.x, marker_xy)
        if not np.any(H):
            return  # singular geometry, skip

        y = self._innov(z, self.x, marker_xy)
        S = H @ self.P @ H.T + self.R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.pub_state.publish(String(data='drop:S_singular'))
            return

        # Mahalanobis gating: reject outliers (occluded / mis-detected marker)
        d2 = float(y.T @ S_inv @ y)
        if d2 > self.gate:
            self._rejects[marker_id] = self._rejects.get(marker_id, 0) + 1
            self.pub_state.publish(
                String(data=f'reject:id={marker_id}:d2={d2:.2f}'))
            return

        K = self.P @ H.T @ S_inv
        dx = K @ y
        # Saturate the corrective step so a one-off bad observation can't
        # teleport the belief. 1 m / 30 deg is generous for a 10 Hz camera.
        dx[0] = float(np.clip(dx[0], -1.0, 1.0))
        dx[1] = float(np.clip(dx[1], -1.0, 1.0))
        dx[2] = float(np.clip(dx[2], -0.5, 0.5))
        self.x = self.x + dx
        self.x[2] = wrap_to_pi(self.x[2])
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
        # Joseph form would be more numerically stable; for 3x3 the simple
        # form is fine in practice and easier to read.

        self._updates[marker_id] = self._updates.get(marker_id, 0) + 1
        self.pub_state.publish(
            String(data=f'update:id={marker_id}:d2={d2:.2f}'))

    # ───────────────────────────────────────────────────────────────────────
    # Metrics (PDF Part 1: "evaluation metrics to verify robustness")
    # ───────────────────────────────────────────────────────────────────────

    def _metrics_tick(self):
        """Imprime cada 1 s las métricas de evaluación del EKF.

        Formato:
            t=12.3s  pose=(x, y, yaw_deg)  gt=(x, y, yaw_deg)
                     err_xy=0.024m  err_yaw=1.8°  trace(P)=0.0031
                     obs=[1,2]  upd={1:53,2:11}  rej={1:0,2:1}
        """
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        x, y, yaw = float(self.x[0]), float(self.x[1]), float(self.x[2])
        trace_p = float(np.trace(self.P))

        if self._gt is not None:
            gx, gy, gyaw = self._gt
            err_xy = math.hypot(x - gx, y - gy)
            err_yaw = math.degrees(wrap_to_pi(yaw - gyaw))
            gt_str = (f'gt=({gx:+.2f}, {gy:+.2f}, '
                      f'{math.degrees(gyaw):+6.1f}°)')
            err_str = f'err_xy={err_xy:.3f}m  err_yaw={err_yaw:+5.1f}°'
        else:
            gt_str = 'gt=N/A'
            err_str = 'err=N/A'

        obs = ','.join(str(i) for i in self._last_obs_ids) or '-'
        upd = '{' + ','.join(f'{k}:{v}' for k, v in sorted(self._updates.items())) + '}'
        rej = '{' + ','.join(f'{k}:{v}' for k, v in sorted(self._rejects.items())) + '}'

        self.get_logger().info(
            f't={t:5.1f}s  pose=({x:+.2f}, {y:+.2f}, '
            f'{math.degrees(yaw):+6.1f}°)  {gt_str}  '
            f'{err_str}  trace(P)={trace_p:.4f}  '
            f'obs=[{obs}]  upd={upd}  rej={rej}'
        )

    # ───────────────────────────────────────────────────────────────────────
    # Output
    # ───────────────────────────────────────────────────────────────────────

    def _publish_map_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for mid in self.aruco_map.ids():
            xy = self.aruco_map.xy(mid)
            yaw = self.aruco_map.yaw(mid) or 0.0
            # Body of the marker (white square seen by the camera).
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'aruco_map'
            m.id = int(mid)
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(xy[0])
            m.pose.position.y = float(xy[1])
            m.pose.position.z = 0.10
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale.x = 0.02
            m.scale.y = 0.18
            m.scale.z = 0.18
            m.color.r = 0.95
            m.color.g = 0.95
            m.color.b = 0.95
            m.color.a = 1.0
            ma.markers.append(m)
            # Text label with the id, floating above.
            t = Marker()
            t.header.frame_id = 'odom'
            t.header.stamp = now
            t.ns = 'aruco_map_label'
            t.id = int(mid)
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(xy[0])
            t.pose.position.y = float(xy[1])
            t.pose.position.z = 0.30
            t.scale.z = 0.15
            t.color.r = 0.2
            t.color.g = 0.9
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = f'id={mid}'
            ma.markers.append(t)
        self.pub_map.publish(ma)

    def _publish_odom(self, v: float, w: float):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.orientation.z = math.sin(float(self.x[2]) / 2.0)
        odom.pose.pose.orientation.w = math.cos(float(self.x[2]) / 2.0)

        # Map 3x3 P -> 6x6 pose.covariance, same layout as Week 5.
        cov = [0.0] * 36
        cov[0]  = float(self.P[0, 0]); cov[1]  = float(self.P[0, 1]); cov[5]  = float(self.P[0, 2])
        cov[6]  = float(self.P[1, 0]); cov[7]  = float(self.P[1, 1]); cov[11] = float(self.P[1, 2])
        cov[30] = float(self.P[2, 0]); cov[31] = float(self.P[2, 1]); cov[35] = float(self.P[2, 2])
        odom.pose.covariance = cov

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)

        self.pub_odom.publish(odom)

        # Broadcast the matching TF so RViz's RobotModel display has a chain
        # all the way down to wheels/sensors (robot_state_publisher chains
        # from base_footprint downward; we provide the odom -> base_footprint
        # link from our EKF belief).
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(self.x[0])
        t.transform.translation.y = float(self.x[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(float(self.x[2]) / 2.0)
        t.transform.rotation.w = math.cos(float(self.x[2]) / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
