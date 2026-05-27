"""
covariance_publisher — Render the EKF covariance as an ellipse for RViz.

RViz's Odometry display draws covariance automatically, but only on the
XY plane and with a fixed style. To satisfy the PDF requirement of
*"plotting confidence ellipsoids ... and their evolution over time"*, this
node consumes /ekf/odom and republishes a Marker (a flat ellipse) on
/ekf/covariance_ellipse, so we can:

  * Tune chi-square confidence (default 5.991 -> 95%, 2 dof).
  * Persist a trail of past ellipses (set keep_trail=True) so the video
    shows how Sigma grows when the marker is occluded and shrinks again
    when a fresh observation lands.

The 2x2 XY block of the pose covariance is decomposed by eigendecomposition;
the ellipse semi-axes are sqrt(chi2 * lambda_i) and the orientation is
atan2(eigvec_max).
"""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class CovariancePublisher(Node):

    def __init__(self):
        super().__init__('covariance_publisher')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('chi2', 5.991)         # 95% confidence, 2 dof
        self.declare_parameter('z_offset', 0.02)
        self.declare_parameter('keep_trail', True)
        self.declare_parameter('trail_length', 60)    # ~6 s @ 10 Hz odom
        self.declare_parameter('trail_period_s', 0.5)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.chi2 = float(self.get_parameter('chi2').value)
        self.z_offset = float(self.get_parameter('z_offset').value)
        self.keep_trail = bool(self.get_parameter('keep_trail').value)
        self.trail_length = int(self.get_parameter('trail_length').value)
        self.trail_period = float(self.get_parameter('trail_period_s').value)

        self.trail = deque(maxlen=self.trail_length)
        self.last_trail_t = self.get_clock().now()
        self.next_id = 0

        self.create_subscription(Odometry, 'ekf/odom', self._odom_cb, 20)
        self.pub = self.create_publisher(
            MarkerArray, 'ekf/covariance_ellipse', 10)

        self.get_logger().info(
            f'CovariancePublisher | chi2={self.chi2} | trail={self.keep_trail}'
        )

    def _odom_cb(self, msg: Odometry):
        cov = list(msg.pose.covariance)
        # 6x6 layout: row-major. XY block lives at indices (0,0),(0,1),(6,0),(6,1)
        # which in the flat array are 0,1,6,7.
        sxx = cov[0]; sxy = cov[1]
        syx = cov[6]; syy = cov[7]
        cov_xy = np.array([[sxx, sxy], [syx, syy]], dtype=float)

        if not np.all(np.isfinite(cov_xy)):
            return

        # Symmetrise then eigendecompose. A near-singular EKF update can
        # produce a 2x2 with eigenvalues like (+1e-3, -1e-20) — tiny negative
        # from floating-point error. Clamp them to 0 before sqrt so a noisy
        # filter never crashes the visualiser.
        cov_xy = 0.5 * (cov_xy + cov_xy.T)
        eigvals, eigvecs = np.linalg.eigh(cov_xy)
        if not np.all(np.isfinite(eigvals)):
            return
        eig_min = max(float(eigvals[0]), 0.0)
        eig_max = max(float(eigvals[1]), 0.0)
        a = math.sqrt(self.chi2 * eig_max)
        b = math.sqrt(self.chi2 * eig_min)
        v = eigvecs[:, 1]
        angle = math.atan2(float(v[1]), float(v[0]))

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        live = self._build_marker(x, y, a, b, angle,
                                   marker_id=999_999,
                                   alpha=0.35, ns='ekf_cov_live')

        ma = MarkerArray()
        ma.markers.append(live)

        if self.keep_trail:
            now = self.get_clock().now()
            dt = (now - self.last_trail_t).nanoseconds * 1e-9
            if dt >= self.trail_period:
                self.last_trail_t = now
                ghost = self._build_marker(x, y, a, b, angle,
                                            marker_id=self.next_id,
                                            alpha=0.12, ns='ekf_cov_trail')
                self.next_id += 1
                self.trail.append(ghost)
            ma.markers.extend(self.trail)

        self.pub.publish(ma)

    def _build_marker(self, x, y, a, b, angle, *,
                      marker_id, alpha, ns):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(self.z_offset)
        m.pose.orientation.z = math.sin(angle / 2.0)
        m.pose.orientation.w = math.cos(angle / 2.0)
        # Cylinder: scale.x = diameter along major, .y diameter along minor,
        # .z very small to flatten it into an ellipse.
        m.scale.x = float(2.0 * a)
        m.scale.y = float(2.0 * b)
        m.scale.z = 0.01
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.6
        m.color.a = float(alpha)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = CovariancePublisher()
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
