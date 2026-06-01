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

import json
import math
import os
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, TransformStamped
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
        # Incertidumbre inicial del estado (P_0). Pequeña porque conocemos el
        # spawn, pero NO cero (ver __init__: P=0 reventaba el NEES).
        self.declare_parameter('sigma_p0_xy', 0.02)    # [m]
        self.declare_parameter('sigma_p0_yaw', 0.0175)  # [rad] ~1 deg
        # Piso de ruido de proceso por tick (desviación estándar). Solo evita
        # que P se vuelva singular si el robot pasa mucho tiempo quieto; se
        # mantiene MUY pequeño porque un piso grande vuelve el filtro
        # conservador (NEES << 3). La consistencia real se logra anulando el
        # sesgo sistemático de range (marker_length calibrado), no inflando Q.
        self.declare_parameter('q_floor_xy', 0.001)    # [m por tick]
        self.declare_parameter('q_floor_yaw', 0.0005)  # [rad por tick]

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
        # Carpeta donde se escribe el reporte final de métricas al cerrar.
        # El launch la fija al docs/ del paquete fuente; vacío = usar cwd.
        self.declare_parameter('report_dir', '')

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
        # P NO debe arrancar en cero: con P=0 el filtro afirma incertidumbre
        # nula, así que cualquier error de mm dispara el NEES (e^T P^-1 e) a
        # miles y la elipse de covarianza arranca como un punto. Sembramos una
        # incertidumbre inicial pequeña pero honesta: sabemos el spawn con
        # ~2 cm en x,y y ~1° en yaw. Esto hace que NEES arranque cerca de 3 y
        # la elipse sea visible desde el primer frame en RViz.
        self.P = np.diag([
            float(gp('sigma_p0_xy')) ** 2,
            float(gp('sigma_p0_xy')) ** 2,
            float(gp('sigma_p0_yaw')) ** 2,
        ])
        # Piso de Q constante (se suma a Q proporcional a velocidad en _tick).
        self.Q_floor = np.diag([
            float(gp('q_floor_xy')) ** 2,
            float(gp('q_floor_xy')) ** 2,
            float(gp('q_floor_yaw')) ** 2,
        ])

        # ── Last wheel velocities (control input) ─────────────────────────────
        self.wr = 0.0
        self.wl = 0.0

        # ── Evaluation metrics state ──────────────────────────────────────────
        # Para el PDF Part 1: error vs ground truth, trace(P), conteos de
        # updates/rejects por marker, RMSE acumulado, NEES (consistencia del
        # filtro). Se imprime cada 1s en _metrics_tick() y un resumen final
        # al destruir el nodo.
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._gt = None                          # último ground_truth
        self._updates = {}                       # marker_id -> count aceptados
        self._rejects = {}                       # marker_id -> count rechazados
        self._consecutive_rejects = {}           # marker_id -> consecutive rejects sin update
        self._last_obs_ids = []                  # ids vistos en el último _aruco_cb
        # Sumas para RMSE y NEES.
        # RMSE_x = sqrt( mean( (x_ekf - x_gt)^2 ) ); igual para y y yaw.
        # NEES_k = e_k^T · P_k^{-1} · e_k    con e_k = x_ekf - x_gt.
        # Para un EKF bien calibrado en 3-DOF (x, y, yaw),
        # E[NEES] ≈ 3 y el 95% de muestras debe caer dentro de
        # chi2(3, 0.025) ≈ 0.216  y  chi2(3, 0.975) ≈ 9.348.
        # Si NEES_mean >> 3 → filtro optimista (subestima incertidumbre).
        # Si NEES_mean << 3 → filtro conservador (sobreestima incertidumbre).
        self._n_samples = 0
        self._sse_x = 0.0           # sum of squared errors en x
        self._sse_y = 0.0
        self._sse_yaw = 0.0
        self._nees_sum = 0.0
        self._nees_in_bounds = 0    # contador de muestras dentro de chi2(3) 95 %
        self._nees_lo = 0.216       # chi2(3, 0.025)
        self._nees_hi = 9.348       # chi2(3, 0.975)

        # Métricas por fase (PDF: 'múltiples / sin / parcial markers').
        # auto_demo publica el id de fase en /demo/phase. Mientras ninguna
        # fase esté activa todas las muestras caen en 'global' (sigue contando
        # como reporte agregado además del por-fase). Cada fase guarda las
        # mismas sumas que el bloque global + un trace_P final para mostrar
        # crecimiento/encogimiento de la elipse entre fases.
        self._phase = None
        self._phases = {}   # phase_id -> dict con sumas
        self._last_trace_P = 0.0

        # ── ROS plumbing ──────────────────────────────────────────────────────
        self.create_subscription(Float32, 'wr', self._wr_cb, 10)
        self.create_subscription(Float32, 'wl', self._wl_cb, 10)
        self.create_subscription(PoseArray, 'aruco/observations',
                                 self._aruco_cb, 10)
        # Ground truth para calcular el error de localización (PDF Part 1:
        # "métricas de evaluación de su algoritmo para verificar robustez").
        self.create_subscription(Odometry, 'ground_truth',
                                 self._gt_cb, 10)
        # Fase del auto_demo (multi_marker / no_marker / reacquire / ...).
        # Segmenta las métricas por escenario del PDF para el reporte final.
        self.create_subscription(String, 'demo/phase',
                                 self._phase_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, 'ekf/odom', 10)
        self.pub_state = self.create_publisher(String, 'ekf/state', 10)
        self.pub_map = self.create_publisher(MarkerArray, 'ekf/aruco_map', 10)
        # /ekf/metrics expone las métricas de evaluación como String JSON-like
        # para que se pueda grabar con `ros2 topic echo` o un rosbag y
        # post-procesar (RMSE, NEES, traza P por timestamp).
        self.pub_metrics = self.create_publisher(String, 'ekf/metrics', 10)
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

    def _phase_cb(self, msg: String):
        """Cambia la fase activa para segmentar las métricas por escenario.

        Cada fase mantiene sus propias sumas (n, sse, nees, updates, rejects)
        en self._phases[phase_id]. La fase '__end__' simplemente la desactiva
        para que las muestras restantes (apagado) no contaminen ninguna.
        """
        pid = (msg.data or '').strip()
        if pid == '__end__':
            self._phase = None
            return
        if not pid:
            return
        if pid not in self._phases:
            self._phases[pid] = {
                'n': 0,
                'sse_x': 0.0, 'sse_y': 0.0, 'sse_yaw': 0.0,
                'nees_sum': 0.0, 'nees_in_bounds': 0,
                'updates': {}, 'rejects': {},
                'trace_P_start': float(self._last_trace_P),
                'trace_P_end': float(self._last_trace_P),
            }
        self._phase = pid

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
        # Piso de ruido de proceso. El Q proporcional a velocidad de rueda se
        # vuelve CERO cuando el robot está quieto (ωr=ωl=0), así que tras varias
        # correcciones seguidas P colapsaba a ~0 y el NEES explotaba (filtro
        # optimista, elipse invisible en RViz). Un Q mínimo constante modela los
        # efectos no capturados por la odometría (deslizamiento, vibración,
        # discretización) que se acumulan AUNQUE el robot no se mueva, y mantiene
        # P en un nivel honesto. Es práctica estándar en EKFs de localización.
        Q = Q + self.Q_floor

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

        # ── Re-acquisition mode ──────────────────────────────────────────────
        # If a marker has been rejected 20+ times consecutively, the EKF
        # belief has likely drifted (e.g. from wheel slip against a wall).
        # In that case, inflate P to acknowledge we're lost and use a much
        # wider gate to allow the observation through. This breaks the
        # death spiral where drift → rejection → more drift → permanent
        # rejection.
        consec = self._consecutive_rejects.get(marker_id, 0)
        effective_gate = self.gate
        if consec >= 20:
            # Inflate covariance — acknowledge we're uncertain.
            inflate = min(2.0, 1.0 + (consec - 20) * 0.05)
            self.P *= inflate
            self.P = 0.5 * (self.P + self.P.T)
            # Widen the gate dramatically to allow re-acquisition.
            effective_gate = self.gate * 50.0
            if consec % 20 == 0:
                self.get_logger().warn(
                    f'RE-ACQ: marker {marker_id} rejected {consec}x '
                    f'consecutively — inflating P (×{inflate:.2f}), '
                    f'widening gate to {effective_gate:.1f}')
            # Recompute S and S_inv with inflated P.
            S = H @ self.P @ H.T + self.R
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return

        # Mahalanobis gating: reject outliers (occluded / mis-detected marker)
        d2 = float(y.T @ S_inv @ y)
        if d2 > effective_gate:
            self._rejects[marker_id] = self._rejects.get(marker_id, 0) + 1
            self._consecutive_rejects[marker_id] = consec + 1
            ph = self._phases.get(self._phase) if self._phase else None
            if ph is not None:
                ph['rejects'][marker_id] = ph['rejects'].get(marker_id, 0) + 1
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

        # Successful update — reset consecutive reject counter.
        self._consecutive_rejects[marker_id] = 0

        self._updates[marker_id] = self._updates.get(marker_id, 0) + 1
        ph = self._phases.get(self._phase) if self._phase else None
        if ph is not None:
            ph['updates'][marker_id] = ph['updates'].get(marker_id, 0) + 1
        self.pub_state.publish(
            String(data=f'update:id={marker_id}:d2={d2:.2f}'))

    # ───────────────────────────────────────────────────────────────────────
    # Metrics (PDF Part 1: "evaluation metrics to verify robustness")
    # ───────────────────────────────────────────────────────────────────────

    def _metrics_tick(self):
        """Imprime cada 1 s las métricas de evaluación del EKF.

        Además acumula RMSE y NEES (Normalized Estimation Error Squared)
        para el reporte final que pide el PDF ("evaluation metrics to
        verify robustness").

        NEES_k = e_k^T · P_k^{-1} · e_k, con e_k = [Δx, Δy, Δyaw].
        Para 3-DOF y filtro consistente: E[NEES] = 3, con bandas chi2
        (0.216, 9.348) cubriendo el 95 % de las muestras.
        """
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        x, y, yaw = float(self.x[0]), float(self.x[1]), float(self.x[2])
        trace_p = float(np.trace(self.P))

        if self._gt is not None:
            gx, gy, gyaw = self._gt
            dx = x - gx
            dy = y - gy
            dyaw = wrap_to_pi(yaw - gyaw)
            err_xy = math.hypot(dx, dy)
            err_yaw_deg = math.degrees(dyaw)

            # Acumular sumas para RMSE (global y por fase activa).
            self._n_samples += 1
            self._sse_x += dx * dx
            self._sse_y += dy * dy
            self._sse_yaw += dyaw * dyaw
            ph = self._phases.get(self._phase) if self._phase else None
            if ph is not None:
                ph['n'] += 1
                ph['sse_x'] += dx * dx
                ph['sse_y'] += dy * dy
                ph['sse_yaw'] += dyaw * dyaw

            # NEES = e^T · P^{-1} · e. Si P es casi singular al inicio,
            # añadimos un epsilon en la diagonal para que la inversa exista.
            P_reg = self.P + 1e-9 * np.eye(3)
            try:
                P_inv = np.linalg.inv(P_reg)
                e = np.array([dx, dy, dyaw], dtype=float)
                nees_k = float(e.T @ P_inv @ e)
                self._nees_sum += nees_k
                in_b = self._nees_lo <= nees_k <= self._nees_hi
                if in_b:
                    self._nees_in_bounds += 1
                if ph is not None:
                    ph['nees_sum'] += nees_k
                    if in_b:
                        ph['nees_in_bounds'] += 1
                nees_str = f'NEES={nees_k:6.2f}'
            except np.linalg.LinAlgError:
                nees_str = 'NEES=NaN'
        # trace(P) snapshot para el reporte por fase (visualiza crecimiento
        # de la elipse durante 'no_marker' y encogimiento en 'reacquire').
        self._last_trace_P = trace_p
        if self._gt is not None:
            if self._phase and self._phase in self._phases:
                self._phases[self._phase]['trace_P_end'] = trace_p
            gt_str = (f'gt=({gx:+.2f}, {gy:+.2f}, '
                      f'{math.degrees(gyaw):+6.1f}°)')
            err_str = (f'err_xy={err_xy:.3f}m  err_yaw={err_yaw_deg:+5.1f}°  '
                       f'{nees_str}')
        else:
            gt_str = 'gt=N/A'
            err_str = 'err=N/A  NEES=N/A'

        obs = ','.join(str(i) for i in self._last_obs_ids) or '-'
        upd = '{' + ','.join(f'{k}:{v}' for k, v in sorted(self._updates.items())) + '}'
        rej = '{' + ','.join(f'{k}:{v}' for k, v in sorted(self._rejects.items())) + '}'

        self.get_logger().info(
            f't={t:5.1f}s  pose=({x:+.2f}, {y:+.2f}, '
            f'{math.degrees(yaw):+6.1f}°)  {gt_str}  '
            f'{err_str}  trace(P)={trace_p:.4f}  '
            f'obs=[{obs}]  upd={upd}  rej={rej}'
        )

        # Publicamos un mensaje compacto en /ekf/metrics para que se
        # pueda capturar a CSV vía `ros2 topic echo` y graficarlo offline.
        if self._gt is not None:
            self.pub_metrics.publish(String(data=(
                f't={t:.2f},x_ekf={x:.4f},y_ekf={y:.4f},yaw_ekf={yaw:.4f},'
                f'x_gt={gx:.4f},y_gt={gy:.4f},yaw_gt={gyaw:.4f},'
                f'err_xy={err_xy:.4f},err_yaw={dyaw:.4f},'
                f'trace_P={trace_p:.6f},NEES={nees_k if "nees_k" in dir() else float("nan"):.4f},'
                f'n_obs={len(self._last_obs_ids)}'
            )))

    def _summarise_phases(self):
        """Convierte self._phases (sumas crudas) en RMSE/NEES por fase.

        Devuelve un dict listo para serializar a JSON. Cada fase incluye su
        tamaño muestral, RMSE, NEES, % en banda, updates/rejects por marker
        y trace(P) al inicio y al final, que es el indicador visual del
        crecimiento/encogimiento de la elipse en RViz entre escenarios.
        """
        out = {}
        for pid, ph in self._phases.items():
            n = ph['n']
            if n == 0:
                continue
            rmse_yaw_deg = math.degrees(math.sqrt(ph['sse_yaw'] / n))
            rmse_xy = math.sqrt((ph['sse_x'] + ph['sse_y']) / n)
            nees_mean = ph['nees_sum'] / n
            in_band_pct = 100.0 * ph['nees_in_bounds'] / n
            upd = sum(ph['updates'].values())
            rej = sum(ph['rejects'].values())
            out[pid] = {
                'samples': n,
                'rmse_xy_m': round(rmse_xy, 4),
                'rmse_yaw_deg': round(rmse_yaw_deg, 3),
                'nees_mean': round(nees_mean, 3),
                'nees_in_band_pct': round(in_band_pct, 1),
                'updates_accepted': upd,
                'updates_rejected': rej,
                'per_marker_accepted': {str(k): v for k, v in sorted(ph['updates'].items())},
                'trace_P_start': round(ph['trace_P_start'], 6),
                'trace_P_end': round(ph['trace_P_end'], 6),
                'trace_P_growth_ratio': round(ph['trace_P_end'] / max(ph['trace_P_start'], 1e-9), 2),
            }
        return out

    def _print_final_summary(self):
        """Resumen final al cerrar el nodo: RMSE acumulado + consistencia NEES.

        Se imprime con un recuadro destacado para que sea fácil de leer
        en la terminal y de capturar en el video del entregable.
        """
        if self._n_samples == 0:
            self.get_logger().warn(
                'No ground truth samples collected — cannot compute RMSE/NEES.')
            return

        n = self._n_samples
        rmse_x = math.sqrt(self._sse_x / n)
        rmse_y = math.sqrt(self._sse_y / n)
        rmse_yaw_deg = math.degrees(math.sqrt(self._sse_yaw / n))
        rmse_xy = math.sqrt((self._sse_x + self._sse_y) / n)
        nees_mean = self._nees_sum / n
        pct_in_bounds = 100.0 * self._nees_in_bounds / n

        total_updates = sum(self._updates.values())
        total_rejects = sum(self._rejects.values())
        accept_pct = (100.0 * total_updates / max(1, total_updates + total_rejects))

        if nees_mean > 3.5:
            consistency = 'OPTIMISTIC (covariance too small)'
        elif nees_mean < 2.5:
            consistency = 'CONSERVATIVE (covariance too large)'
        else:
            consistency = 'CONSISTENT — filter is well-tuned'

        # Dict estructurado: única fuente de verdad para el .txt y el .json.
        # Así el reporte legible y el parseable nunca se desincronizan.
        metrics = {
            'challenge': 'Part 1 — EKF Visual Localisation',
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'observation_model': self.model_name,
            'config': {
                'sample_time_s': self.dt,
                'wheel_radius_m': self.r,
                'wheelbase_m': self.l,
                'k_r': self.k_r,
                'k_l': self.k_l,
                'mahalanobis_gate': self.gate,
            },
            'samples': n,
            'rmse': {
                'xy_m': round(rmse_xy, 4),
                'x_m': round(rmse_x, 4),
                'y_m': round(rmse_y, 4),
                'yaw_deg': round(rmse_yaw_deg, 3),
            },
            'nees': {
                'mean': round(nees_mean, 3),
                'expected': 3.0,
                'in_band_pct': round(pct_in_bounds, 1),
                'band_chi2_3dof_95': [self._nees_lo, self._nees_hi],
                'consistency': consistency,
            },
            'aruco_updates': {
                'accepted': total_updates,
                'rejected': total_rejects,
                'accept_pct': round(accept_pct, 1),
                'per_marker_accepted': {str(k): v for k, v in sorted(self._updates.items())},
                'per_marker_rejected': {str(k): v for k, v in sorted(self._rejects.items())},
            },
            # Métricas por fase del auto_demo (escenarios del PDF: multi-marker
            # / sin marker / re-adquisición / etc.). Vacío si no hubo señal en
            # /demo/phase (corrida manual sin auto_demo).
            'phases': self._summarise_phases(),
        }
        self._last_metrics = metrics

        bar = '═' * 72
        lines = [
            '',
            f'╔{bar}╗',
            '║  PART 1 · EKF Localisation — FINAL EVALUATION METRICS' + ' ' * 18 + '║',
            f'╠{bar}╣',
            f'║  Samples              : {n:>6d}                                              ║',
            f'║  RMSE position (xy)   : {rmse_xy:>6.3f} m                                       ║',
            f'║    RMSE x             : {rmse_x:>6.3f} m                                       ║',
            f'║    RMSE y             : {rmse_y:>6.3f} m                                       ║',
            f'║  RMSE yaw             : {rmse_yaw_deg:>6.2f} °                                       ║',
            '║                                                                        ║',
            f'║  NEES mean            : {nees_mean:>6.2f}   (expected ≈ 3.00 for 3-DOF filter)  ║',
            f'║  NEES in 95% χ² band  : {pct_in_bounds:>5.1f}%  (band = [0.216, 9.348])           ║',
            '║                                                                        ║',
            f'║  ArUco updates accepted: {total_updates:>5d}                                          ║',
            f'║  ArUco updates rejected: {total_rejects:>5d}  ({100-accept_pct:.1f}%)' + ' ' * 30 + '║',
            f'║  Per-marker breakdown  : upd={dict(sorted(self._updates.items()))}   rej={dict(sorted(self._rejects.items()))}',
            f'╚{bar}╝',
            '',
            '  Interpretation:',
            f'    • RMSE_xy = {rmse_xy:.3f} m   → average position error vs Gazebo ground truth',
            f'    • NEES = {nees_mean:.2f}     → {consistency}',
            f'    • {pct_in_bounds:.1f}% of samples inside the χ²(3) 95 % band',
            '',
        ]
        # Sub-bloque por fase si hubo auto_demo (PDF: escenarios robustos).
        phases_summary = metrics['phases']
        if phases_summary:
            lines.append('  Per-phase breakdown (PDF robustness scenarios):')
            lines.append(f'  {"phase":<15}{"n":>5}{"RMSE_xy":>10}{"NEES":>8}{"in_band":>9}{"upd":>5}{"rej":>5}{"P_grow":>8}')
            for pid, ps in phases_summary.items():
                lines.append(
                    f'  {pid:<15}{ps["samples"]:>5}'
                    f'{ps["rmse_xy_m"]:>10.4f}{ps["nees_mean"]:>8.2f}'
                    f'{ps["nees_in_band_pct"]:>8.1f}%'
                    f'{ps["updates_accepted"]:>5}{ps["updates_rejected"]:>5}'
                    f'{ps["trace_P_growth_ratio"]:>8.2f}x'
                )
            lines.append('')
        for line in lines:
            self.get_logger().info(line)

        # Además del log en terminal, persistimos el reporte en dos formatos:
        #   .txt  — recuadro legible para capturar en el video.
        #   .json — estructurado y parseable para graficar/citar en el reporte.
        self._write_report_file(lines, metrics)

    def _write_report_file(self, lines, metrics):
        """Guarda el resumen final como .txt (legible) y .json (estructurado).

        En runtime el nodo corre desde install/, así que __file__ NO sirve
        para llegar al docs/ del fuente. Permitimos fijar el destino con el
        parámetro ROS `report_dir` (lo setea el launch al path del fuente);
        si no está, caemos al cwd (el workspace, de donde se lanza ros2) y por
        último a /tmp. Cualquier fallo se reporta por log, sin tirar el nodo.
        Ambos archivos comparten timestamp para emparejarlos fácilmente.
        """
        stamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        report_dir = (self.get_parameter('report_dir').value
                      if self.has_parameter('report_dir') else '')
        candidates = [d for d in (report_dir, os.path.join(os.getcwd(), 'part1_reports'), '/tmp') if d]
        for target_dir in candidates:
            try:
                os.makedirs(target_dir, exist_ok=True)
                txt_path = os.path.join(target_dir, f'report_part1_{stamp}.txt')
                json_path = os.path.join(target_dir, f'report_part1_{stamp}.json')
                with open(txt_path, 'w', encoding='utf-8') as f:
                    f.write('\n'.join(lines))
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(metrics, f, indent=2, ensure_ascii=False)
                self.get_logger().info(f'Report saved (txt): {txt_path}')
                self.get_logger().info(f'Report saved (json): {json_path}')
                return
            except OSError as e:
                self.get_logger().warn(f'Could not write report to {target_dir}: {e}')
        self.get_logger().error('Failed to persist the Part 1 report anywhere.')

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
        # Reporte final de métricas (RMSE + NEES) antes de cerrar — es lo
        # que el PDF Part 1 pide como "evaluation metrics to verify
        # robustness". Se imprime aunque haya Ctrl-C porque va en finally.
        try:
            node._print_final_summary()
        except Exception as exc:
            node.get_logger().warn(f'Could not print final summary: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
