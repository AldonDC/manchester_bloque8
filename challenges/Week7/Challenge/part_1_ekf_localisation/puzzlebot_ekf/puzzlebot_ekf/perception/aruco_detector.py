"""
aruco_detector — Detect ArUco markers from a camera stream and publish their
range+bearing in the robot frame.

Pipeline per frame:
    1. Subscribe to /camera/image_raw (sensor_msgs/Image)
    2. Convert with cv_bridge -> grayscale
    3. Detect markers with the selected ArUco dictionary
    4. Estimate pose per marker with cv2.aruco.estimatePoseSingleMarkers
       using K, D from /camera/camera_info (or a YAML override)
    5. Transform marker translation from camera frame to robot frame
       via a static T_robot_cam (params)
    6. Publish (id, range, bearing) per marker via PoseArray on
       /aruco/observations  (encoding: position.x=id, .y=range, .z=bearing)
    7. Optionally draw axes on the image and publish to /aruco/image

Why PoseArray as a carrier
--------------------------
We could have defined a custom .msg, but adding new packages bloats the
build for the Final Challenge. PoseArray ships with every ROS 2 install,
and three floats are enough for (id, r, phi). The convention is documented
here and in ekf_node._aruco_cb.

Library policy
--------------
Only OpenCV (incl. aruco), NumPy, cv_bridge (ROS-side bridge, not a third-
party math lib) — within the challenge rules.
"""

import math
from typing import Optional

import cv2 as cv
import numpy as np
from cv2 import aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(
        "cv_bridge is required. Install with: sudo apt install ros-${ROS_DISTRO}-cv-bridge"
    ) from e


_ARUCO_DICTS = {
    '4x4_50':   aruco.DICT_4X4_50,
    '4x4_100':  aruco.DICT_4X4_100,
    '4x4_250':  aruco.DICT_4X4_250,
    '4x4_1000': aruco.DICT_4X4_1000,
    '5x5_100':  aruco.DICT_5X5_100,
    '5x5_250':  aruco.DICT_5X5_250,
    '6x6_250':  aruco.DICT_6X6_250,
}


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('aruco_dict', '4x4_50')
        self.declare_parameter('marker_length', 0.10)
        self.declare_parameter('publish_debug_image', True)

        # Static transform robot <- camera (XYZ + yaw of camera in robot frame).
        # Defaults assume the camera is mounted forward of base_link, looking
        # along +X. Tune these to match your URDF / real robot mount.
        self.declare_parameter('cam_x_in_robot', 0.10)
        self.declare_parameter('cam_y_in_robot', 0.00)
        self.declare_parameter('cam_z_in_robot', 0.10)
        self.declare_parameter('cam_yaw_in_robot', 0.0)

        # Optional intrinsics override (skip /camera_info if a YAML is given).
        self.declare_parameter('camera_intrinsics_yaml', '')

        dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        if dict_name not in _ARUCO_DICTS:
            raise ValueError(
                f"Unknown aruco_dict '{dict_name}'. "
                f"Valid: {sorted(_ARUCO_DICTS.keys())}"
            )
        self.dictionary = aruco.getPredefinedDictionary(_ARUCO_DICTS[dict_name])

        # API shim: OpenCV >= 4.7 exposes aruco.ArucoDetector (class-based,
        # detector.detectMarkers(img)). OpenCV 4.5 (Ubuntu 22.04 apt) only has
        # the function-based API: aruco.detectMarkers(img, dict, parameters=).
        # We pick whichever exists so the node runs on both.
        if hasattr(aruco, 'ArucoDetector'):
            self._detector_params = aruco.DetectorParameters()
            self._detector_obj = aruco.ArucoDetector(
                self.dictionary, self._detector_params)
            self._detect = lambda gray: self._detector_obj.detectMarkers(gray)
        else:
            # OpenCV 4.5.x — function API with DetectorParameters_create().
            self._detector_params = aruco.DetectorParameters_create() \
                if hasattr(aruco, 'DetectorParameters_create') \
                else aruco.DetectorParameters()
            self._detect = lambda gray: aruco.detectMarkers(
                gray, self.dictionary, parameters=self._detector_params)

        self.marker_length = float(self.get_parameter('marker_length').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        # Robot-frame camera offset
        self.cam_tx = float(self.get_parameter('cam_x_in_robot').value)
        self.cam_ty = float(self.get_parameter('cam_y_in_robot').value)
        self.cam_tz = float(self.get_parameter('cam_z_in_robot').value)
        self.cam_yaw = float(self.get_parameter('cam_yaw_in_robot').value)

        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        intr_yaml = self.get_parameter('camera_intrinsics_yaml').get_parameter_value().string_value
        if intr_yaml:
            self._load_intrinsics_from_yaml(intr_yaml)

        self.bridge = CvBridge()
        # QoS depth=1: solo nos importa el frame MÁS RECIENTE. Si el detector
        # va lento (procesa a 5-10Hz y la cámara publica a 30Hz), saltarse
        # frames viejos es MEJOR que acumularlos (queue baja CPU usage en
        # la cola interna y reduce latencia del overlay).
        self.create_subscription(Image, 'camera/image_raw', self._image_cb, 1)
        self.create_subscription(CameraInfo, 'camera/camera_info',
                                 self._cam_info_cb, 1)
        self.pub_obs = self.create_publisher(
            PoseArray, 'aruco/observations', 10)
        self.pub_dbg = self.create_publisher(Image, 'aruco/image', 10) \
            if self.publish_debug else None

        api = 'ArucoDetector (>=4.7)' if hasattr(aruco, 'ArucoDetector') \
            else 'function API (<4.7)'
        self.get_logger().info(
            f'ArucoDetector | cv2={cv.__version__} api={api} | '
            f'dict={dict_name} side={self.marker_length}m | '
            f'cam@robot=({self.cam_tx:.2f},{self.cam_ty:.2f},{self.cam_tz:.2f}) '
            f'yaw={self.cam_yaw:.2f}'
        )
        self._last_log_t = 0.0
        self._frames_seen = 0
        self._frames_with_marker = 0

    # ─────────────────────────────────────────────────────────────────────────

    def _load_intrinsics_from_yaml(self, path: str):
        import yaml
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.K = np.array(data['camera_matrix'], dtype=float)
        self.D = np.array(data['dist_coeff'], dtype=float)
        self.get_logger().info(f'Loaded intrinsics from YAML: {path}')

    def _cam_info_cb(self, msg: CameraInfo):
        if self.K is not None:
            return  # YAML override wins; only seed once
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.D = np.array(msg.d, dtype=float)
        self.get_logger().info('Camera intrinsics seeded from /camera/camera_info')

    def _image_cb(self, msg: Image):
        if self.K is None:
            return  # waiting for intrinsics
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        self._frames_seen += 1
        corners, ids, _ = self._detect(gray)

        # Throttled status (1 Hz) so we can verify the camera + detector are
        # alive even before the robot sees anything.
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            rate = self._frames_with_marker / max(self._frames_seen, 1)
            self.get_logger().info(
                f'frames={self._frames_seen} with_marker={self._frames_with_marker} '
                f'({100*rate:.0f}%) last_ids={None if ids is None else ids.flatten().tolist()}'
            )

        if ids is None or len(ids) == 0:
            if self.pub_dbg is not None:
                self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
            return

        self._frames_with_marker += 1
        obs = PoseArray()
        obs.header.stamp = msg.header.stamp
        obs.header.frame_id = 'base_link'

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = aruco.estimatePoseSingleMarkers(
                corners[i], self.marker_length, self.K, self.D)[:2]
            tvec = tvec[0][0]  # (x, y, z) in camera frame, OpenCV convention

            # Camera optical frame: +X right, +Y down, +Z forward.
            # Robot frame:          +X forward, +Y left, +Z up.
            # Map: x_robot_cam = +Z_cam ; y_robot_cam = -X_cam ; z_robot_cam = -Y_cam
            x_rel_cam = float(tvec[2])
            y_rel_cam = -float(tvec[0])

            # Then apply the static cam-in-robot transform (translation + yaw).
            cy = math.cos(self.cam_yaw)
            sy = math.sin(self.cam_yaw)
            x_rel = self.cam_tx + cy * x_rel_cam - sy * y_rel_cam
            y_rel = self.cam_ty + sy * x_rel_cam + cy * y_rel_cam

            r = math.hypot(x_rel, y_rel)
            phi = math.atan2(y_rel, x_rel)

            # Sanity gate: solvePnP can return wildly wrong poses when the
            # marker is at an extreme viewing angle, partly occluded, or when
            # only a tiny fraction of the marker is visible. We bail on any
            # range outside a plausible 0.05 m -- 6 m window (our arena is
            # 7 m across, so anything farther is obviously bogus).
            if not (0.05 < r < 6.0) or not math.isfinite(phi):
                continue

            p = Pose()
            p.position.x = float(marker_id)
            p.position.y = float(r)
            p.position.z = float(phi)
            obs.poses.append(p)

            if self.pub_dbg is not None:
                aruco.drawDetectedMarkers(img, [corners[i]], np.array([[marker_id]]))
                cv.drawFrameAxes(img, self.K, self.D, rvec, tvec, 0.05)
                c = corners[i].reshape(4, 2).mean(axis=0).astype(int)
                cv.putText(img, f'id={marker_id} r={r:.2f}m phi={math.degrees(phi):.0f}deg',
                           (c[0], c[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 255, 0), 2)

        self.pub_obs.publish(obs)
        if self.pub_dbg is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
