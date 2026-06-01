"""
astar_nav_node — Nodo ROS2 que orquesta A* + Catmull-Rom + Pure Pursuit.

Arquitectura del PDF Final Challenge MCR2 implementada con técnicas de
robótica clásica (sin librerías externas):

    [Mapa YAML laberinto] → OccupancyGrid (numpy)
                                    │
    [Lista de waypoints PDF] → A* desde cero → path crudo
                                    │
                              Catmull-Rom smoother
                                    │
                              Pure Pursuit tracker
                                    │ /cmd_vel
                              Puzzlebot

    [/ekf/odom from Part 1] → pose corregida → tracker

JUSTIFICACIÓN DE FRECUENCIAS:
    Control loop @ 50 Hz (20ms): suficiente para PID/Pure Pursuit fluido
        en Puzzlebot. El controlador interno del simulador integra a
        1kHz pero las correcciones de yaw cada 20ms son visualmente
        smooth.

    Planning loop @ 1 Hz: la planificación A* es offline (mapa conocido,
        no cambia). Re-planificamos sólo si llegamos al goal y queremos
        ir al siguiente, o si el waypoint manager nos da uno nuevo.

ESTADOS FSM:
    IDLE     → esperando primera pose.
    PLANNING → corriendo A* hacia el waypoint actual.
    EXECUTING → siguiendo el path con Pure Pursuit.
    ARRIVED  → en el waypoint, espera dwell, luego pide el siguiente.
"""

from __future__ import annotations

import math
import os
import time
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid as RosOccupancyGrid
from std_msgs.msg import String, Header, Float32
from visualization_msgs.msg import Marker, MarkerArray

from puzzlebot_nav.planning.occupancy_grid import OccupancyGrid, grid_from_yaml
from puzzlebot_nav.planning.astar_planner import astar
from puzzlebot_nav.planning.path_smoother import smooth as smooth_path
from puzzlebot_nav.control.pure_pursuit import PurePursuit, PurePursuitParams


class FsmState(str, Enum):
    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
    EXECUTING = 'EXECUTING'
    ARRIVED = 'ARRIVED'
    FINISHED = 'FINISHED'


class AstarNavNode(Node):
    """Nodo de navegación A* + Pure Pursuit para Part 2 MCR2."""

    def __init__(self) -> None:
        super().__init__('astar_nav')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('control_rate', 30.0)  # 33ms — fluido sin saturar
        self.declare_parameter('planning_rate', 1.0)
        # Pure Pursuit (tuneado conservador post-test):
        #   lookahead 0.30m: el robot sigue el path MÁS fielmente, no
        #     atajar curvas (con 0.40m atajaba esquinas y chocaba).
        #   v_max 0.18 m/s: más lento → menos overshoot al frenar en
        #     puntos críticos (esquinas de chambers).
        #   goal_tolerance 0.25: generoso, el robot puede declarar "llegué"
        #     un poco antes para que el waypoint manager avance suave.
        self.declare_parameter('lookahead_distance', 0.30)
        self.declare_parameter('max_linear_vel', 0.18)
        self.declare_parameter('max_angular_vel', 0.80)  # rotar suave =
                                                          # cámara estable
                                                          # = ArUco detection
        self.declare_parameter('goal_tolerance', 0.30)  # generoso para
                                                         # llegada robusta
        self.declare_parameter('brake_distance', 0.50)
        # Misión
        self.declare_parameter(
            'waypoints',
            [2.5, 0.5,
             5.2, 2.5,
             2.5, 3.5,
             -0.5, 1.5])
        self.declare_parameter('loop', True)
        self.declare_parameter('dwell_time', 1.0)
        # Spawn (para inicialización si no llega odom).
        self.declare_parameter('x_init', 0.9)
        self.declare_parameter('y_init', 0.8)
        self.declare_parameter('theta_init', 1.5708)
        # Frame
        self.declare_parameter('frame_id', 'odom')

        # ── Cargar mapa ───────────────────────────────────────────────────────
        map_yaml_path = self.get_parameter('map_yaml').value
        if not map_yaml_path or not os.path.exists(map_yaml_path):
            raise RuntimeError(
                f'map_yaml inválido: {map_yaml_path!r}. '
                'Pasa la ruta absoluta del YAML del mapa.')
        with open(map_yaml_path) as f:
            map_data = yaml.safe_load(f)
        self.grid = grid_from_yaml(map_data)
        self.get_logger().info(f'Mapa cargado: {self.grid}')

        # ── Cargar waypoints ──────────────────────────────────────────────────
        flat = list(self.get_parameter('waypoints').value)
        if len(flat) < 8 or len(flat) % 2 != 0:
            raise ValueError(
                f'waypoints debe ser flat list de >=8 floats. Got {len(flat)}.')
        self.waypoints: List[Tuple[float, float]] = [
            (float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)
        ]
        self.loop = bool(self.get_parameter('loop').value)
        self.dwell_time = float(self.get_parameter('dwell_time').value)
        self.get_logger().info(
            f'Misión: {len(self.waypoints)} waypoints, loop={self.loop}')

        # ── Pure Pursuit ──────────────────────────────────────────────────────
        gp = lambda n: self.get_parameter(n).value
        pp_params = PurePursuitParams(
            lookahead_distance=float(gp('lookahead_distance')),
            max_linear_vel=float(gp('max_linear_vel')),
            max_angular_vel=float(gp('max_angular_vel')),
            goal_tolerance=float(gp('goal_tolerance')),
            brake_distance=float(gp('brake_distance')),
        )
        self.tracker = PurePursuit(pp_params)

        # ── Estado FSM ────────────────────────────────────────────────────────
        self.state = FsmState.IDLE
        self.wp_idx = 0
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        # Pose actual del robot (vía topic 'odom', remapeado a /ground_truth
        # en el launch — esta es la pose REAL del simulador que usa la nav).
        self.x = float(gp('x_init'))
        self.y = float(gp('y_init'))
        self.yaw = float(gp('theta_init'))
        self._pose_received = False
        # Pose EKF (vía /ekf/odom — pose ESTIMADA con ArUcos). Se usa SOLO
        # para visualización: el cilindro azul y la flecha azul se dibujan
        # sobre esta pose para que coincidan EXACTAMENTE con la flecha roja
        # del EKF Odom y con la elipse de covarianza. Así RViz se ve coherente:
        #   - RobotModel (URDF) → posición REAL Gazebo (vía /tf)
        #   - Cilindro azul + flecha roja + elipse → posición ESTIMADA EKF
        # El offset visible entre uno y otro ES la incertidumbre del EKF.
        self.ekf_x = float(gp('x_init'))
        self.ekf_y = float(gp('y_init'))
        self.ekf_yaw = float(gp('theta_init'))
        self._ekf_pose_received = False
        # Tiempo de llegada al waypoint actual (para dwell).
        self._arrived_at: Optional[float] = None
        # Historial de waypoints visitados (para validación de secuencia).
        self._visited: list = []
        self._laps_completed = 0
        # ── Safety LiDAR layer ────────────────────────────────────────────
        # Distancias del LiDAR (3 sectores publicados por lidar_processor).
        # safety_distance: si frente < esto, frenamos antes de chocar.
        self.d_front = float('inf')
        self.d_left = float('inf')
        self.d_right = float('inf')
        # safety_distance MINIMO (0.15m). El robot mide 10cm de radio.
        # Con 0.15m solo frena si literalmente va a chocar. El A* path
        # ya garantiza 28cm de margen contra paredes en el grid inflado,
        # así que safety_distance solo actúa como red de seguridad
        # cuando hay drift severo de tracking. Antes con 0.22m frenaba
        # espuriamente al pasar cerca de iv3_west o iv4_west durante
        # los tramos diagonales del path A*.
        self.declare_parameter('safety_distance', 0.15)
        self.safety_distance = float(self.get_parameter('safety_distance').value)
        # Histéresis MÍNIMA: solo 5cm de margen para liberar.
        self._safety_release = self.safety_distance + 0.05
        self._safety_active = False

        # ── ROS topics ────────────────────────────────────────────────────────
        # QoS especial para topics "latched" (RViz se conecta tarde):
        #   occupancy_grid: TRANSIENT_LOCAL para que el último mensaje
        #     persista en el publisher, así RViz lo recibe aunque se conecte
        #     después de que se publicó. Sin esto el grid NO aparece en RViz
        #     porque se publica solo cada 1s y RViz puede perderse el envío.
        #   planned_path: igual razón pero menos crítico.
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, 'planned_path', latched_qos)
        self.pub_grid = self.create_publisher(
            RosOccupancyGrid, 'occupancy_grid', latched_qos)
        self.pub_state = self.create_publisher(String, 'astar_nav/state', 10)
        self.pub_wpm = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # Suscribimos a odom. Internamente el remap en el launch decide si es
        # /odom (encoders crudos) o /ekf/odom (pose corregida con ArUcos).
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        # ALINEACIÓN VISUAL: el robot azul se dibuja sobre la pose EKF para
        # que coincida con la flecha roja del EKF Odom y con la elipse de
        # covarianza. Suscribimos a /ekf/odom con path absoluto para no
        # depender del remap (que afecta solo a 'odom' → /ground_truth).
        self.create_subscription(Odometry, '/ekf/odom', self._ekf_odom_cb, 10)
        # LiDAR sectorizado para layer de safety. Si el LiDAR frontal
        # ve obstáculo cerca, el control frena antes de chocar.
        self.create_subscription(Float32, 'bug/d_front',
                                 lambda m: setattr(self, 'd_front', m.data), 5)
        self.create_subscription(Float32, 'bug/d_left',
                                 lambda m: setattr(self, 'd_left', m.data), 5)
        self.create_subscription(Float32, 'bug/d_right',
                                 lambda m: setattr(self, 'd_right', m.data), 5)

        # Timers — optimizados para no saturar RViz.
        # control_rate sigue alto (50Hz) porque es el lazo de control real.
        # Visualizaciones bajadas para liberar CPU/GPU:
        #   grid → cada 5s (es estático/latched, no necesita más)
        #   waypoint markers → cada 1s (active naranja sigue visible <1s
        #     después de avanzar al siguiente WP)
        control_rate = float(gp('control_rate'))
        planning_rate = float(gp('planning_rate'))
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_tick)
        self.planning_timer = self.create_timer(1.0 / planning_rate, self._planning_tick)
        # Grid estático con QoS latched: 1 vez cada 5s basta.
        self.create_timer(5.0, self._publish_grid)
        # Waypoint markers a 1Hz: rápido para feedback pero sin saturar.
        self.create_timer(1.0, self._publish_waypoint_markers)
        # Marker del Puzzlebot REACTIVADO como cilindro rojo + flecha,
        # SUSTITUYE al RobotModel (URDF) que es muy pesado: 13 links
        # × shader Ogre cada frame en software rendering = lag y el
        # mesh complejo "se ve" atravesando paredes por el lag de TF.
        # Un solo cilindro Marker (20cm diámetro, exacto al robot real)
        # renderiza 13× más rápido y nunca se ve fuera de lugar.
        self.create_timer(0.066, self._publish_robot_marker)
        self.pub_robot_viz = self.create_publisher(
            MarkerArray, 'robot_viz', 10)

        self.get_logger().info(
            f'A* nav up: control@{control_rate:.0f}Hz, plan@{planning_rate:.0f}Hz, '
            f'lookahead={pp_params.lookahead_distance}m, '
            f'v_max={pp_params.max_linear_vel}m/s'
        )

        # Publicar grid + waypoint markers INMEDIATAMENTE al arrancar.
        # Con latched QoS el grid persiste para cuando RViz se conecte.
        # Los waypoints (banderas verdes) deben aparecer desde t=0 para
        # que el usuario vea visualmente las metas antes de moverse.
        self._publish_grid()
        self._publish_waypoint_markers()

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(
                f'Primera pose: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f}°)'
            )

    def _ekf_odom_cb(self, msg: Odometry) -> None:
        """Pose EKF para el visual azul (queda alineado con flecha roja)."""
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.ekf_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y
        if not self._ekf_pose_received:
            self._ekf_pose_received = True
            self.get_logger().info(
                f'EKF pose recibida: ({self.ekf_x:.2f}, {self.ekf_y:.2f}, '
                f'{math.degrees(self.ekf_yaw):.0f}°) — robot azul ahora sigue al EKF'
            )

    # ── Loop de planificación (1 Hz) ─────────────────────────────────────────

    def _planning_tick(self) -> None:
        """Decide qué hacer según el estado FSM."""
        if self.state == FsmState.IDLE and self._pose_received:
            self._start_planning_to_current_wp()
        elif self.state == FsmState.ARRIVED:
            now = time.time()
            if self._arrived_at is not None and (now - self._arrived_at) >= self.dwell_time:
                # Avanzar al siguiente waypoint.
                self._advance_waypoint()

    def _start_planning_to_current_wp(self) -> None:
        """Planifica A* desde la pose actual hasta self.waypoints[self.wp_idx].

        Si A* falla, NO salta automáticamente al siguiente waypoint.
        En su lugar, reintenta hasta 3 veces (cada planning_tick) y
        marca el waypoint como FALLIDO solo si TODOS los intentos fallan.
        Esto evita el bug donde un fallo transitorio de A* hacía que
        el robot saltara waypoints en la secuencia.
        """
        self.state = FsmState.PLANNING
        goal = self.waypoints[self.wp_idx]
        self.get_logger().info(
            f'Planning A* desde ({self.x:.2f},{self.y:.2f}) → '
            f'wp[{self.wp_idx}]={goal}'
        )
        raw_path = astar(self.grid, (self.x, self.y), goal)
        if raw_path is None:
            # Contador de retries por waypoint (init al primer fallo).
            if not hasattr(self, '_plan_retries'):
                self._plan_retries = 0
            self._plan_retries += 1
            self.get_logger().warn(
                f'A*: no se encontró path a wp[{self.wp_idx}]={goal} '
                f'(intento {self._plan_retries}/3)'
            )
            if self._plan_retries >= 3:
                self.get_logger().error(
                    f'A*: tras 3 intentos sin path a wp[{self.wp_idx}], '
                    f'lo marco como inalcanzable y avanzo.'
                )
                self._plan_retries = 0
                self._advance_waypoint()
            else:
                # Volver a IDLE para que el planning_tick reintente.
                self.state = FsmState.IDLE
            return
        # Path OK → reset contador de retries.
        self._plan_retries = 0
        # Suavizado denso: 25 puntos por segmento Catmull-Rom = path
        # ~3cm entre muestras. Con lookahead 0.30m el Pure Pursuit ve
        # ~10 puntos en su ventana → tracking suave sin atajar curvas.
        smoothed = smooth_path(raw_path, points_per_segment=25)
        self.tracker.set_path(smoothed)
        self._publish_planned_path(smoothed)
        self.state = FsmState.EXECUTING
        self.get_logger().info(
            f'Path listo: {len(raw_path)} celdas → {len(smoothed)} smoothed → EXECUTING')

    def _advance_waypoint(self) -> None:
        """Avanza al siguiente waypoint en orden ESTRICTO.

        Marca el waypoint actual como visitado y aumenta wp_idx EN 1.
        NO salta waypoints bajo ninguna circunstancia.

        DETENCIÓN al cierre del lap (cumplido el PDF Part 2):
        Cuando el último waypoint (idx == n-1) marca llegada y debería
        volver al wp[0] inicial, declaramos MISIÓN COMPLETADA y paramos
        el robot. El PDF pide "trayectoria cerrada visitando 4 puntos
        objetivo" — un lap basta. Loop infinito solo sirve para debug.
        """
        old_idx = self.wp_idx
        # Marcar visitado.
        self._visited.append(old_idx)
        # AVANCE ESTRICTO: idx + 1.
        self.wp_idx += 1
        if self.wp_idx >= len(self.waypoints):
            # ¡LAP COMPLETO! El PDF pide trayectoria cerrada — listo.
            self._laps_completed += 1
            self.state = FsmState.FINISHED
            self.get_logger().info(
                f'═══════════════════════════════════════════════════════')
            self.get_logger().info(
                f'  🎯 MISIÓN COMPLETADA · LAP {self._laps_completed}')
            self.get_logger().info(
                f'  Trayectoria cerrada visitada: {self._visited}')
            self.get_logger().info(
                f'  Robot detenido en wp[{old_idx}]={self.waypoints[old_idx]}')
            self.get_logger().info(
                f'═══════════════════════════════════════════════════════')
            # Frenar inmediatamente.
            self._publish_cmd(0.0, 0.0)
            return
        else:
            self.get_logger().info(
                f'═══ ADVANCE: wp[{old_idx}]={self.waypoints[old_idx]} → '
                f'wp[{self.wp_idx}]={self.waypoints[self.wp_idx]} ═══')
        self.state = FsmState.IDLE  # Trigger replan next planning_tick
        self._arrived_at = None

    # ── Loop de control (50 Hz) ──────────────────────────────────────────────

    def _control_tick(self) -> None:
        """Comanda /cmd_vel según el estado."""
        self._publish_state()

        if self.state != FsmState.EXECUTING:
            # En cualquier estado que no sea EXECUTING, freno.
            self._publish_cmd(0.0, 0.0)
            return

        pose = (self.x, self.y, self.yaw)
        if self.tracker.is_goal_reached(pose):
            self._publish_cmd(0.0, 0.0)
            self.state = FsmState.ARRIVED
            self._arrived_at = time.time()
            self.get_logger().info(
                f'ARRIVED wp[{self.wp_idx}]={self.waypoints[self.wp_idx]}, '
                f'dwell {self.dwell_time:.1f}s'
            )
            return

        # ── LiDAR safety layer ───────────────────────────────────────────
        # Si el LiDAR frontal ve pared muy cerca, intercepta el comando
        # del Pure Pursuit y aplica freno + giro suave al lado más libre.
        # Esto evita choques físicos cuando el tracking del path tiene
        # error (overshoot, drift de odometría).
        v, w = self.tracker.compute_control(pose)
        v, w = self._apply_safety(v, w)
        self._publish_cmd(v, w)

    def _apply_safety(self, v_cmd: float, w_cmd: float) -> Tuple[float, float]:
        """Layer LiDAR-aware: frena/desvía si hay pared muy cerca.

        DESHABILITADA cerca del goal (dist < 0.50m): cuando el robot está
        casi llegando, el LiDAR puede ver la bandera/marker visual del
        waypoint como obstáculo. Si dejáramos el safety activo, el robot
        nunca completaría el tramo final. La regla de oro: confiar en
        el path A* cerca del goal porque está validado contra el grid
        inflado (que no incluye banderas decorativas).

        Lógica con histéresis:
          - Si d_front < safety_distance (0.30m) Y v_cmd > 0 Y dist_goal > 0.5m
            → activamos safety (frena + gira al lado libre).
          - Sale cuando d_front > safety_release (0.45m) o cerca del goal.
        """
        # Solo intercepta si vamos hacia adelante (no afecta giros en sitio).
        if v_cmd <= 0.01:
            self._safety_active = False
            return v_cmd, w_cmd

        # CRÍTICO: si estamos cerca del waypoint actual, NO activamos safety.
        # El LiDAR puede detectar el goal_flag (bandera visual) o el marker
        # ArUco del waypoint como obstáculo. Confiamos en el path A* en este
        # tramo final porque fue validado contra el grid inflado.
        goal_x, goal_y = self.waypoints[self.wp_idx]
        dist_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)
        # Umbral de "cerca del goal" — más generoso para asegurar que
        # cuando estamos en el tramo final NO frenamos por banderas/markers.
        GOAL_PROXIMITY = 0.70
        if dist_to_goal < GOAL_PROXIMITY:
            if self._safety_active:
                self.get_logger().info(
                    f'Safety auto-release: cerca del goal '
                    f'(dist={dist_to_goal:.2f}m)')
            self._safety_active = False
            return v_cmd, w_cmd

        if self._safety_active:
            # Salir solo si está claramente libre.
            if self.d_front > self._safety_release:
                self._safety_active = False
                self.get_logger().info(
                    f'Safety release: d_front={self.d_front:.2f}m')
                return v_cmd, w_cmd
        else:
            if self.d_front < self.safety_distance:
                self._safety_active = True
                self.get_logger().warn(
                    f'SAFETY STOP: d_front={self.d_front:.2f}m '
                    f'(safety={self.safety_distance:.2f}m). '
                    f'd_left={self.d_left:.2f} d_right={self.d_right:.2f} '
                    f'dist_goal={dist_to_goal:.2f}m'
                )

        if self._safety_active:
            # Freno + giro al lado más libre. Si front cerca pero
            # alguno de los lados está libre, giramos hacia ahí.
            v_safe = 0.0
            if self.d_left >= self.d_right:
                w_safe = 0.6  # gira a la izquierda
            else:
                w_safe = -0.6  # gira a la derecha
            return v_safe, w_safe

        return v_cmd, w_cmd

    # ── Publicación ──────────────────────────────────────────────────────────

    def _publish_cmd(self, v: float, w: float) -> None:
        # Cap final de seguridad.
        v = max(-0.15, min(0.35, float(v)))
        w = max(-2.5, min(2.5, float(w)))
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    def _publish_state(self) -> None:
        self.pub_state.publish(String(data=self.state.value))

    def _publish_planned_path(self, path: List[Tuple[float, float]]) -> None:
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _publish_grid(self) -> None:
        msg = RosOccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.cols
        msg.info.height = self.grid.rows
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0
        # Aplanar grid → int8 0/100.
        arr = self.grid.to_ros_occupancy().flatten().tolist()
        msg.data = arr
        self.pub_grid.publish(msg)

    def _publish_robot_marker(self) -> None:
        """Marker AZUL del Puzzlebot (cilindro + flecha de orientación).

        Usa la pose REAL del simulador (ground_truth, vía remap 'odom').
        Razón: si usábamos /ekf/odom el cilindro azul aparecía DENTRO de
        paredes cuando el EKF tenía drift (visualmente parecía "choque"
        en RViz aunque en Gazebo el robot estaba en el pasillo). La
        flecha roja del display EKF Odom sigue mostrando la estimación
        del EKF — ahí se VE el drift contra el azul, que es justamente
        el punto pedagógico de la Sección B.
        """
        if not self._pose_received:
            return
        x, y, yaw = self.x, self.y, self.yaw

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        # Cuerpo del Puzzlebot (cilindro rojo, 18cm diámetro = robot real).
        body = Marker()
        body.header.frame_id = self.frame_id
        body.header.stamp = now
        body.ns = 'puzzlebot_body'
        body.id = 0
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = float(x)
        body.pose.position.y = float(y)
        body.pose.position.z = 0.05
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = 0.18
        body.scale.z = 0.10
        body.color.r = 0.90
        body.color.g = 0.15
        body.color.b = 0.15
        body.color.a = 0.95
        ma.markers.append(body)
        # Flecha de orientación (rojo oscuro).
        arrow = Marker()
        arrow.header.frame_id = self.frame_id
        arrow.header.stamp = now
        arrow.ns = 'puzzlebot_arrow'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = float(x)
        arrow.pose.position.y = float(y)
        arrow.pose.position.z = 0.11
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x = 0.20
        arrow.scale.y = 0.04
        arrow.scale.z = 0.04
        arrow.color.r = 0.60
        arrow.color.g = 0.05
        arrow.color.b = 0.05
        arrow.color.a = 1.0
        ma.markers.append(arrow)
        self.pub_robot_viz.publish(ma)

    def _publish_waypoint_markers(self) -> None:
        """Banderas verdes en los waypoints (igual que waypoint_manager)."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        GREEN = (0.10, 0.85, 0.20, 1.0)
        ORANGE = (1.00, 0.55, 0.05, 1.0)

        for i, (x, y) in enumerate(self.waypoints):
            active = (i == self.wp_idx and self.state != FsmState.FINISHED)
            color = ORANGE if active else GREEN
            # Disco en el suelo.
            disc = Marker()
            disc.header.frame_id = self.frame_id
            disc.header.stamp = now
            disc.ns = 'wp_disc'
            disc.id = i
            disc.type = Marker.CYLINDER
            disc.action = Marker.ADD
            disc.pose.position.x = float(x)
            disc.pose.position.y = float(y)
            disc.pose.position.z = 0.01
            disc.scale.x = disc.scale.y = 0.40
            disc.scale.z = 0.02
            (disc.color.r, disc.color.g, disc.color.b, disc.color.a) = color
            ma.markers.append(disc)
            # Esfera elevada.
            ball = Marker()
            ball.header.frame_id = self.frame_id
            ball.header.stamp = now
            ball.ns = 'wp_ball'
            ball.id = i
            ball.type = Marker.SPHERE
            ball.action = Marker.ADD
            ball.pose.position.x = float(x)
            ball.pose.position.y = float(y)
            ball.pose.position.z = 0.55
            r = 0.20 if active else 0.16
            ball.scale.x = ball.scale.y = ball.scale.z = r
            (ball.color.r, ball.color.g, ball.color.b, ball.color.a) = color
            ma.markers.append(ball)
            # Label.
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = 'wp_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = 0.85
            label.scale.z = 0.20
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'WP{i}' + (' ◄ active' if active else '')
            ma.markers.append(label)

        # Línea cerrada.
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = 'wp_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.04
        (line.color.r, line.color.g, line.color.b, line.color.a) = (0.10, 0.85, 0.20, 0.55)
        pts = list(self.waypoints) + ([self.waypoints[0]] if self.loop else [])
        for x, y in pts:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.04
            line.points.append(p)
        ma.markers.append(line)

        self.pub_wpm.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = AstarNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
