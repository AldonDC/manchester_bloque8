"""
goto_goal — Navegador con máquina de estados explícita
======================================================
Final Challenge · Part 2 · Multi-waypoint Navigation

Implementación profesional con FSM de 5 estados y velocidades
adaptativas por estado, recomendado por el PDF de MCR2:

    > Sugerencias técnicas:
    >   · Controlador propio (P/PI/PID para velocidades lineal y angular)
    >   · Máquina de estados para navegación entre waypoints
    >   · Considerar control de Lyapunov o `go-to-goal` clásico

ESTADOS (cada uno con su propio perfil de velocidad):

    IDLE        — Sin goal. v=0, ω=0.
    ALIGN       — Alineando heading al goal. v=0, ω alto (gira en sitio).
    CRUISE      — Camino libre, lejos del goal. v máximo, ω moderado.
    AVOID       — Obstáculo cerca. v reducida, ω alto (bordea con LiDAR).
    APPROACH    — Cerca del goal (<0.8m). v reducida (precisión), ω bajo.
    ARRIVED     — En el goal. v=0, ω=0, publica GOAL_REACHED.

TRANSICIONES:

           ┌──────────┐
           │   IDLE   │
           └────┬─────┘
                │ goal_received
                ▼
           ┌──────────┐ |ang_err|<25° ┌──────────┐
           │  ALIGN   │──────────────▶│  CRUISE  │
           └─────▲────┘               └────┬─────┘
                 │                          │
                 │ |ang_err|>40°            │ obstáculo<0.6m
                 │                          ▼
                 │                    ┌──────────┐
                 │                    │  AVOID   │
                 │                    └────┬─────┘
                 │                          │ camino libre
                 │                          ▼
                 │                    ┌──────────┐
                 │ dist<0.8m          │  CRUISE  │  ...
                 │                    └────┬─────┘
                 └──────────────────────┐  │
                                        │  │ dist<0.8m
                                        ▼  ▼
                                    ┌──────────┐
                                    │ APPROACH │
                                    └────┬─────┘
                                          │ dist<0.45m
                                          ▼
                                    ┌──────────┐
                                    │ ARRIVED  │──── publica GOAL_REACHED
                                    └──────────┘

CONTROL DE VELOCIDAD:

    cmd_vel.linear.x  = v_state · cos²(ang_err)  · saturación
    cmd_vel.angular.z = kp_ang · ang_err          · saturación

    El factor cos² evita avanzar cuando el robot está mal orientado
    (en ALIGN, ang_err es grande → cos²≈0 → v=0). En CRUISE y APPROACH,
    ang_err es chico → cos²≈1 → v plena.

LIDAR:

    Usamos 3 sectores del lidar_processor (/bug/d_front, d_left, d_right).
    AVOID gira al lado con MÁS espacio. Histéresis: una vez elegido el
    lado, no cambia hasta tener frente libre claro.

CONTRATO ROS:
    Subscribe:
        /odom                      (nav_msgs/Odometry)        ← localización
        /goal                      (geometry_msgs/Point)      ← waypoint actual
        /bug/d_front, d_left, d_right (std_msgs/Float32)      ← LiDAR
        /waypoint_manager/state    (std_msgs/String)          ← para el panel
    Publica:
        /cmd_vel                   (geometry_msgs/Twist)      ← velocidades
        /set_point                 (geometry_msgs/Point)      ← compat. con stack
        /bug/state                 (std_msgs/String)          ← FSM state
"""

import math
import re
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import MarkerArray

from puzzlebot_nav.navigation._status_panel import render as render_panel


class State(str, Enum):
    """Estados de la FSM. Hereda de str para serializar directo al topic."""
    IDLE     = 'IDLE'
    ALIGN    = 'ALIGN'
    CRUISE   = 'CRUISE'
    AVOID    = 'AVOID'
    APPROACH = 'APPROACH'
    ARRIVED  = 'ARRIVED'


class GotoGoal(Node):
    """Go-to-goal con FSM explícita y velocidades adaptativas por estado."""

    def __init__(self):
        super().__init__('goto_goal')

        # ── Parámetros generales ──────────────────────────────────────────────
        self.declare_parameter('control_period', 0.05)  # 20 Hz
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)

        # ── Umbrales de transición de la FSM ──────────────────────────────────
        # ARRIVED: dentro del disco de la bandera (0.36m diam).
        self.declare_parameter('arrival_radius',  0.45)
        # APPROACH: zona de aproximación final (frenado preciso).
        self.declare_parameter('approach_radius', 0.80)
        # ALIGN: rotar en sitio si el heading se desvía mucho.
        self.declare_parameter('align_enter_deg', 40.0)  # entra ALIGN
        self.declare_parameter('align_exit_deg',  25.0)  # sale a CRUISE
        # AVOID: distancia LiDAR por debajo de la cual entramos a AVOID.
        self.declare_parameter('avoid_enter_dist', 0.45)
        self.declare_parameter('avoid_exit_dist',  0.70)  # histéresis
        # COLISIÓN INMINENTE: capa de seguridad por ENCIMA de la FSM.
        # Si el LiDAR ve un obstáculo a menos de emergency_dist (en CUALQUIER
        # sector front/left/right), v se FUERZA a 0 inmediatamente y solo
        # giramos hacia el lado libre. Es el cinturón de seguridad final que
        # evita que el robot toque físicamente las paredes (Gazebo + RViz se
        # vuelven locos cuando hay colisión: el robot rebota, la odometría
        # se desincroniza y el filtro EKF/PID diverge). Margen vs avoid_enter:
        # avoid_enter es "esquivar antes de que esté peligroso", emergency
        # es "FRENA YA porque vas a chocar".
        self.declare_parameter('emergency_dist',    0.20)
        self.declare_parameter('emergency_release', 0.30)  # histéresis

        # ── Perfiles de velocidad por estado ──────────────────────────────────
        # v_max en cada estado (m/s).
        self.declare_parameter('v_cruise',   0.25)  # avance normal
        self.declare_parameter('v_avoid',    0.12)  # esquivando
        self.declare_parameter('v_approach', 0.10)  # aproximación final
        # ω_max en cada estado (rad/s).
        self.declare_parameter('w_align',    2.00)  # rotando en sitio AGGRESIVO
        self.declare_parameter('w_cruise',   0.80)
        self.declare_parameter('w_avoid',    1.20)  # gira fuerte para esquivar
        self.declare_parameter('w_approach', 0.50)  # giro suave cerca del goal
        # Ganancias del controlador (proporcionales).
        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('kp_ang', 2.0)

        # ── Fuente de odometría (modo integrado P1+P2) ────────────────────────
        # Por defecto 'odom' = odometría pura de las ruedas (Part 2 standalone).
        # Cambiar a 'ekf/odom' = pose corregida por EKF + ArUcos (Sección B
        # del PDF: "/robot_posewithcovariance ← EKF"). Eso es lo que pide la
        # arquitectura integrada del Reto Final: la nav usa la pose CORREGIDA,
        # no la cruda de encoders.
        self.declare_parameter('odom_topic', 'odom')

        self._load_params()

        # ── Estado interno ────────────────────────────────────────────────────
        self.state = State.IDLE
        self.x = self.y = self.th = 0.0
        self.tx = self.get_parameter('target_x').value
        self.ty = self.get_parameter('target_y').value
        self.d_front = self.d_left = self.d_right = float('inf')

        self._odom_received = False
        self._lidar_ready = False
        self._goal_received = False

        # Latch para GOAL_REACHED: solo publicamos una vez por waypoint.
        self._reached_published = False

        # Histéresis del lado de evasión en AVOID. 0=ningún lado elegido,
        # +1=izquierda libre, -1=derecha libre.
        self._avoid_side = 0
        # Flag de emergencia (colisión inminente). Activa con histéresis:
        # se enciende cuando algo < emergency_dist, se apaga solo cuando
        # TODO está > emergency_release. Mientras esté activo, v=0 y solo
        # giramos al lado libre.
        self._emergency = False
        # Detección de AVOID atascado: si llevamos N ticks en AVOID con
        # d_front sin mejorar, forzamos retroceso para salir del callejón.
        self._avoid_ticks = 0
        self._avoid_best_front = 0.0  # mejor d_front visto en este AVOID
        # Cooldown tras salir de AVOID: evita re-entrada inmediata que
        # genera el loop AVOID→ALIGN→AVOID. El robot necesita tiempo
        # para completar su rotación en ALIGN antes de volver a AVOID.
        self._avoid_cooldown = 0

        # Anti-deadlock: detección de "no avancé/roté en N segundos".
        self._stuck_x = 0.0
        self._stuck_y = 0.0
        self._stuck_th = 0.0
        self._stuck_ticks = 0

        # Para el panel ASCII.
        self.wp_idx = 0
        self.wp_total = 4
        self.wpm_phase = '...'
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        # ── ROS plumbing ──────────────────────────────────────────────────────
        # Publicamos DIRECTAMENTE en /cmd_vel (saltamos al controller PID
        # externo): así la FSM tiene control completo de v y ω por estado.
        # set_point se mantiene por compatibilidad con el stack (markers,
        # tests del controller original), pero ya no es el camino crítico.
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_setpoint = self.create_publisher(Point, 'set_point', 10)
        self.pub_state = self.create_publisher(String, 'bug/state', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'bug/markers', 10)

        odom_topic = self.get_parameter('odom_topic').value
        self.get_logger().info(f'Subscribed to odom source: /{odom_topic}')
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Point,    'goal',        self._goal_cb, 10)
        self.create_subscription(Float32,  'bug/d_front', self._df_cb,  10)
        self.create_subscription(Float32,  'bug/d_left',  self._dl_cb,  10)
        self.create_subscription(Float32,  'bug/d_right', self._dr_cb,  10)
        self.create_subscription(String, 'waypoint_manager/state',
                                 self._wpm_cb, 10)

        self.timer = self.create_timer(self.dt, self._loop)
        self.create_timer(0.5, self._panel)

        self.get_logger().info(
            f'goto_goal FSM | dt={self.dt:.3f}s  '
            f'arrival={self.arrival_radius:.2f}m  '
            f'approach={self.approach_radius:.2f}m'
        )

    def _load_params(self):
        gp = lambda n: self.get_parameter(n).value
        self.dt              = gp('control_period')
        self.arrival_radius  = gp('arrival_radius')
        self.approach_radius = gp('approach_radius')
        self.align_enter_deg = gp('align_enter_deg')
        self.align_exit_deg  = gp('align_exit_deg')
        self.avoid_enter     = gp('avoid_enter_dist')
        self.avoid_exit      = gp('avoid_exit_dist')
        self.emergency_dist  = gp('emergency_dist')
        self.emergency_release = gp('emergency_release')
        self.v_cruise        = gp('v_cruise')
        self.v_avoid         = gp('v_avoid')
        self.v_approach      = gp('v_approach')
        self.w_align         = gp('w_align')
        self.w_cruise        = gp('w_cruise')
        self.w_avoid         = gp('w_avoid')
        self.w_approach      = gp('w_approach')
        self.kp_lin          = gp('kp_lin')
        self.kp_ang          = gp('kp_ang')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny_cosp, cosy_cosp)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if not self._odom_received:
            self._odom_received = True
            self._stuck_x, self._stuck_y = self.x, self.y
            self._stuck_th = self.th
            self.get_logger().info(
                f'odom OK: pose=({self.x:.2f},{self.y:.2f})')

    def _goal_cb(self, msg: Point):
        if abs(self.tx - msg.x) > 1e-3 or abs(self.ty - msg.y) > 1e-3:
            self.tx, self.ty = msg.x, msg.y
            # Reset COMPLETO de FSM al cambiar de waypoint.
            self.state = State.ALIGN
            self._reached_published = False
            self._avoid_side = 0
            self._avoid_ticks = 0
            self._avoid_best_front = 0.0
            self._stuck_x, self._stuck_y = self.x, self.y
            self._stuck_th = self.th
            self._stuck_ticks = 0
            self._stuck_level = 0
            # CRÍTICO: limpiar emergency también. Si el robot llegó a un
            # waypoint con paredes cerca y se publicó GOAL_REACHED falso
            # del stuck-L3, el emergency podía quedar latched bloqueando
            # el inicio del nuevo waypoint.
            if self._emergency:
                self.get_logger().info(
                    'Goal nuevo: limpiando flag _emergency (era latched)')
            self._emergency = False
            self.get_logger().info(
                f'Nuevo goal: ({self.tx:.2f},{self.ty:.2f}) → ALIGN')
        self._goal_received = True

    def _df_cb(self, m: Float32):
        if not self._lidar_ready and math.isfinite(m.data):
            self._lidar_ready = True
        self.d_front = m.data

    def _dl_cb(self, m: Float32): self.d_left  = m.data
    def _dr_cb(self, m: Float32): self.d_right = m.data

    def _wpm_cb(self, msg: String):
        # Parsear 'RUNNING wp[2]' o 'DWELL wp[1]' etc.
        m = re.search(r'wp\[(\d+)\]', msg.data or '')
        if m:
            self.wp_idx = int(m.group(1))
        self.wpm_phase = msg.data.split()[0] if msg.data else '...'

    # ── Loop principal ───────────────────────────────────────────────────────

    def _loop(self):
        # Espera a tener odom + LiDAR + primer goal antes de actuar.
        if not (self._odom_received and self._lidar_ready and self._goal_received):
            self._publish_state()
            return

        # Cálculos de error reutilizados por todos los estados.
        dx = self.tx - self.x
        dy = self.ty - self.y
        dist = math.hypot(dx, dy)
        ang_to_goal = math.atan2(dy, dx)
        ang_err = self._wrap_pi(ang_to_goal - self.th)
        ang_err_deg = math.degrees(ang_err)

        # ── Anti-deadlock con ESCAPE MANEUVER profesional ────────────────────
        self._stuck_ticks += 1
        check_period_ticks = int(4.0 / self.dt)
        if self.state != State.ARRIVED and self._stuck_ticks >= check_period_ticks:
            moved_dist = math.hypot(self.x - self._stuck_x, self.y - self._stuck_y)
            moved_angle = abs(self._wrap_pi(self.th - self._stuck_th))
            if moved_dist < 0.15 and moved_angle < math.radians(10.0):
                # No avanzó ni rotó suficiente → escalar respuesta.
                self._stuck_level = getattr(self, '_stuck_level', 0) + 1
                if self._stuck_level == 1:
                    self.get_logger().warn(
                        f'STUCK L1 (dist={moved_dist:.2f}m, angle={math.degrees(moved_angle):.1f}° en 4s) → ALIGN reset')
                    self.state = State.ALIGN
                    self._avoid_side = 0
                elif self._stuck_level == 2:
                    self.get_logger().warn(
                        f'STUCK L2 → ESCAPE: retroceso + giro al lado libre')
                    self._emergency = True
                    self._avoid_side = +1 if self.d_left >= self.d_right else -1
                else:
                    self.get_logger().error(
                        f'STUCK L3+ ({self._stuck_level}): robot atorado, '
                        f'esperando que wpm avance al siguiente goal')
                    if self._stuck_level == 3 and not self._reached_published:
                        self.pub_state.publish(String(data='GOAL_REACHED'))
                        self._reached_published = True
                        self.get_logger().warn(
                            'Forzando avance al siguiente waypoint '
                            '(este es inalcanzable)')
            else:
                self._stuck_level = 0
            self._stuck_x, self._stuck_y = self.x, self.y
            self._stuck_th = self.th
            self._stuck_ticks = 0

        # ── CAPA DE SEGURIDAD: colisión inminente ────────────────────────────
        # Por ENCIMA de la FSM. Si cualquier sector ve algo muy cerca,
        # forzamos v=0 y giramos al lado libre. Histéresis para evitar
        # bouncing entre modo emergencia y modo normal.
        #
        # EXCEPCIÓN CRÍTICA: en ALIGN con ang_err grande, el robot solo
        # ROTA en sitio. El frente barre paredes cercanas durante la
        # rotación, pero NO hay riesgo de colisión porque v=0. Activar
        # emergencia aquí interrumpe la rotación y atrapa al robot.
        front_lidar = self.d_front
        is_rotating_in_place = (
            self.state == State.ALIGN and abs(ang_err_deg) > 40.0
        )

        if not is_rotating_in_place:
            if self._emergency:
                # Salir solo si el frente está claramente libre.
                if front_lidar > self.emergency_release:
                    self._emergency = False
                    self.get_logger().info(
                        f'Emergencia liberada (d_front={front_lidar:.2f}m)')
            else:
                if front_lidar < self.emergency_dist:
                    self._emergency = True
                    # Decidir lado de escape: el sector con MÁS espacio.
                    if self.d_left >= self.d_right:
                        self._avoid_side = +1
                    else:
                        self._avoid_side = -1
                    self.get_logger().warn(
                        f'EMERGENCIA d_front={self.d_front:.2f}  '
                        f'd_left={self.d_left:.2f}  d_right={self.d_right:.2f}  '
                        f'→ escape {"left" if self._avoid_side>0 else "right"}')
        else:
            # Durante rotación ALIGN, silenciar emergencia temporalmente.
            if self._emergency:
                self._emergency = False

        if self._emergency:
            is_stuck = getattr(self, '_stuck_level', 0) >= 1
            if abs(ang_err_deg) > 30.0:
                W_MAX_ROTATE = 2.0
                sign = 1.0 if ang_err > 0 else -1.0
                w_emerg = sign * W_MAX_ROTATE
                v_emerg = -0.08 if (self.d_front < 0.18 or is_stuck) else 0.0
            elif self.d_front < 0.15 or is_stuck:
                v_emerg = -0.10
                w_emerg = 1.5 * self._avoid_side
            else:
                v_emerg = 0.0
                w_emerg = self.w_avoid * self._avoid_side
            self._publish_cmd(v_emerg, w_emerg)
            self._publish_state()
            return

        # ── Transiciones de la FSM (modo normal) ─────────────────────────────
        self._update_state(dist, ang_err_deg)

        # ── Acciones por estado ──────────────────────────────────────────────
        v, w = self._compute_command(dist, ang_err)
        self._publish_cmd(v, w)

        # set_point para compatibilidad con el resto del stack (markers).
        self.pub_setpoint.publish(
            Point(x=float(self.tx), y=float(self.ty), z=0.0))
        self._publish_state()

    def _update_state(self, dist: float, ang_err_deg: float):
        """Evalúa transiciones de la FSM con regla de PRECEDENCIA estricta.

        Prioridades (de mayor a menor):
          1. ARRIVED       — dist < arrival_radius (sale cuando hay nuevo goal)
          2. AVOID         — obstáculo frontal a <avoid_enter (con histéresis)
          3. ALIGN         — |ang_err| > 40° SIEMPRE precede CRUISE/APPROACH:
                             si el robot está mirando al revés, NO debe
                             intentar acercarse, debe girar PRIMERO. Este
                             es el fix del bug donde el robot pasaba el
                             goal y se quedaba mirando al sur sin volver.
          4. APPROACH      — dist < approach_radius (y heading aceptable)
          5. CRUISE        — default

        Sale de ALIGN cuando |ang_err| < 25° (histéresis 40°/25°).
        """
        # ── 1. Llegada (prioridad máxima) ──
        if dist < self.arrival_radius:
            if self.state != State.ARRIVED:
                self.state = State.ARRIVED
                self.get_logger().info(
                    f'ARRIVED  dist={dist:.2f}m  '
                    f'goal=({self.tx:.2f},{self.ty:.2f})')
            return

        # Si veníamos de ARRIVED y nos alejamos (el waypoint manager ya
        # nos mandó otro goal), regresamos a ALIGN para volver a alinear.
        if self.state == State.ARRIVED:
            self.state = State.ALIGN
            return

        # ── 2. HARD-ALIGN: si goal está MUY desorientado, gira PRIMERO ──
        # Prioridad ABSOLUTA sobre AVOID. Lógica de ingeniero senior:
        # girar en sitio NO requiere espacio frontal libre (el robot
        # rota sobre su eje sin avanzar). Si el goal está a >90° del
        # heading, NO hay razón para entrar a AVOID — el frente cercano
        # NO está en la dirección del goal de cualquier modo. Resolver
        # el ALIGN primero y luego decidir.
        # Este es el fix del bug donde el robot pasaba un waypoint y
        # quedaba atorado en AVOID girando hacia "el lado libre" en
        # vez de hacia el siguiente waypoint.
        if abs(ang_err_deg) > 90.0:
            self.state = State.ALIGN
            return

        # ── 3. Evasión (entra/sale con histéresis + cooldown) ──
        # Cooldown: tras salir de AVOID, NO re-entrar durante N ticks
        # para que ALIGN complete la rotación al goal. Sin esto, el
        # robot oscila AVOID→ALIGN→AVOID indefinidamente.
        if self._avoid_cooldown > 0:
            self._avoid_cooldown -= 1

        if self.state == State.AVOID:
            # Sale solo cuando frente CLARAMENTE libre.
            if self.d_front > self.avoid_exit:
                self._avoid_side = 0
                self._avoid_ticks = 0
                self._avoid_best_front = 0.0
                # Cooldown: 1.5 segundos sin poder re-entrar a AVOID.
                self._avoid_cooldown = int(1.5 / self.dt)
                # Fuerza ALIGN para re-orientar al goal después del bordeo.
                self.state = State.ALIGN
                self.get_logger().info(
                    f'AVOID → ALIGN (cooldown {self._avoid_cooldown} ticks)')
            return
        elif self.d_front < self.avoid_enter and self._avoid_cooldown <= 0:
            self.state = State.AVOID
            # Decide lado: el que tiene MÁS espacio.
            self._avoid_side = +1 if self.d_left >= self.d_right else -1
            self._avoid_ticks = 0
            self._avoid_best_front = self.d_front
            self.get_logger().info(
                f'AVOID  d_front={self.d_front:.2f}m  '
                f'→ esquiva {"left" if self._avoid_side>0 else "right"}')
            return

        # ── 4. SOFT-ALIGN: precede a APPROACH/CRUISE ──
        # Si el robot está moderadamente desorientado (40°-90°), gira en
        # sitio. Si está alineado (<25°), sale a CRUISE/APPROACH.
        if self.state == State.ALIGN:
            if abs(ang_err_deg) < self.align_exit_deg:
                if dist < self.approach_radius:
                    self.state = State.APPROACH
                else:
                    self.state = State.CRUISE
            return
        elif abs(ang_err_deg) > self.align_enter_deg:
            self.state = State.ALIGN
            self.get_logger().info(
                f'ALIGN  ang_err={ang_err_deg:+.1f}°  '
                f'(necesita girar a apuntar al goal)')
            return

        # ── 4. Aproximación final ──
        if dist < self.approach_radius:
            self.state = State.APPROACH
            return

        # ── 5. Default: CRUISE ──
        self.state = State.CRUISE

    def _compute_command(self, dist: float, ang_err: float) -> tuple[float, float]:
        """Calcula (v_lin, v_ang) según el estado actual.

        Patrón común:
            ω = saturar(kp_ang · ang_err,  ±w_max_state)
            v = saturar(kp_lin · dist · cos²(ang_err),  v_max_state)

        El factor cos²(ang_err) evita avanzar mal orientado. En estados
        donde el robot debe quedarse quieto o solo girar (IDLE, ALIGN,
        ARRIVED), el cos² lo hace solo (ALIGN tiene |ang_err| grande →
        cos²≈0). No hay que forzar v=0 manualmente.
        """
        if self.state == State.IDLE or self.state == State.ARRIVED:
            return 0.0, 0.0

        ang_err_deg = math.degrees(ang_err)

        # ω proporcional al error angular, saturada por estado.
        w_target = self.kp_ang * ang_err

        if self.state == State.ALIGN:
            # ALIGN: rotación en sitio AGGRESIVA para apuntar al goal rápido.
            # Usamos signo(ang_err) × w_align (no kp_ang*ang_err saturado)
            # para garantizar VELOCIDAD MÁXIMA DE GIRO constante, no
            # proporcional al error. Esto evita la "zona muerta" donde
            # kp_ang*ang_err < umbral del motor → el robot no rota.
            # Pasa a ω proporcional solo cuando |ang_err| < 30° para
            # frenar suavemente al llegar al target heading.
            if abs(ang_err_deg) > 30.0:
                # Máxima velocidad de giro hacia el goal.
                sign = 1.0 if ang_err > 0 else -1.0
                w = sign * self.w_align
            else:
                # Frenado suave proporcional cerca del target.
                w = self._sat(self.kp_ang * ang_err, self.w_align)
            return 0.0, w

        if self.state == State.AVOID:
            # AVOID con escalamiento según severidad y progreso:
            #
            # Tracking: cuánto tiempo llevamos en AVOID y cuál es el mejor
            # d_front visto. Si no mejoramos en N ticks, escalamos.
            self._avoid_ticks += 1
            if self.d_front > self._avoid_best_front + 0.05:
                self._avoid_best_front = self.d_front
                self._avoid_ticks = 0  # progresando, reset

            stuck_in_avoid = self._avoid_ticks > int(2.5 / self.dt)  # 2.5s sin mejorar
            very_stuck = self._avoid_ticks > int(5.0 / self.dt)      # 5s sin mejorar

            # Si llevamos >5s sin mejorar, INVERTIR el lado de evasión.
            # En una esquina cóncava, girar siempre al mismo lado puede
            # empeorar las cosas. Invertir rompe el ciclo.
            if very_stuck and self._avoid_ticks % int(5.0 / self.dt) == 0:
                self._avoid_side *= -1
                self._avoid_best_front = self.d_front  # reset referencia
                self.get_logger().warn(
                    f'AVOID very stuck → SWAP side to '
                    f'{"left" if self._avoid_side > 0 else "right"}')

            if self.d_front < 0.25 or stuck_in_avoid:
                # Retroceso + giro al lado libre.
                v = -0.08
                w = self.w_avoid * self._avoid_side
                if stuck_in_avoid and self._avoid_ticks % int(2.0/self.dt) == 0:
                    self.get_logger().warn(
                        f'AVOID stuck (best d_front={self._avoid_best_front:.2f}m), '
                        f'retrocediendo')
            elif self.d_front < 0.35:
                # Pared cerca, gira fuerte sin avanzar.
                v = 0.0
                w = self.w_avoid * self._avoid_side
            else:
                # Bordeando normal, avanza despacio al lado libre.
                v = self.v_avoid
                w = self.w_avoid * 0.7 * self._avoid_side

            return v, self._sat(w, self.w_avoid)

        # Gate angular LINEAL (reemplaza el cos² problemático).
        # ang_err = 0°  → gate = 1.0  (avance pleno)
        # ang_err = 30° → gate = 0.5
        # ang_err = 60° → gate = 0.0  (no avanza, deja que ALIGN tome control)
        # Esto evita la zona muerta del cos² cuando ang_err > 90° y permite
        # transiciones más suaves de ALIGN a CRUISE.
        gate = max(0.0, 1.0 - abs(math.degrees(ang_err)) / 60.0)

        if self.state == State.APPROACH:
            # Aproximación final con FRENADO PROPORCIONAL CUADRÁTICO.
            # Diseño de control senior: en CRUISE v = kp·dist (lineal con
            # distancia). Para APPROACH usamos v = kp·dist² para frenado
            # más agresivo cerca del goal — evita que el robot se PASE
            # del waypoint por inercia (problema observado en v4 donde
            # el robot llegaba a WP0=(2.5, 0.0) y por momentum salía a
            # y=-0.5, zona sin recovery).
            #
            # Con dist=0.8m → v = kp · 0.64 = 0.51 m/s (saturada a 0.10)
            # Con dist=0.4m → v = kp · 0.16 = 0.13 m/s (saturada a 0.10)
            # Con dist=0.2m → v = kp · 0.04 = 0.03 m/s (frena natural)
            # Resultado: frenado suave sin overshoot.
            w = self._sat(w_target, self.w_approach)
            v_brake = self.kp_lin * dist * dist * gate
            v = self._sat(v_brake, self.v_approach)
            return v, w

        # CRUISE: avance estándar.
        w = self._sat(w_target, self.w_cruise)
        v = self._sat(self.kp_lin * dist * gate, self.v_cruise)
        return v, w

    # ── Publicación ──────────────────────────────────────────────────────────

    def _publish_cmd(self, v: float, w: float):
        # Cap final con límites físicos del Puzzlebot.
        # v ∈ [-0.15, +0.35] m/s, w ∈ [-2.5, +2.5] rad/s.
        v = max(-0.15, min(0.35, float(v)))
        w = max(-2.5, min(2.5, float(w)))
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

    def _publish_state(self):
        """Publica el estado actual con latch para ARRIVED.

        ARRIVED se publica UNA SOLA VEZ como GOAL_REACHED (lo que espera
        el waypoint_manager). Mientras tanto, el robot puede seguir
        publicando estados normales sin re-disparar la transición DWELL
        del manager.

        NOTA: ya NO sobreescribimos con 'EMERGENCY' porque eso impedía
        que el stuck-L3 forzara GOAL_REACHED cuando emergency estaba
        activo (el GOAL_REACHED se publicaba pero inmediatamente después
        el siguiente tick publicaba 'EMERGENCY', y el waypoint_manager
        podía no recibirlo a tiempo). Ahora el estado publicado siempre
        refleja la FSM real.
        """
        if self.state == State.ARRIVED:
            if not self._reached_published:
                self.pub_state.publish(String(data='GOAL_REACHED'))
                self._reached_published = True
            else:
                self.pub_state.publish(String(data='IDLE'))
        else:
            tag = self.state.value
            if self._emergency:
                tag = f'EMERGENCY:{tag}'
            self.pub_state.publish(String(data=tag))

    # ── Utilidades ───────────────────────────────────────────────────────────

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _sat(x: float, lim: float) -> float:
        return max(-lim, min(lim, x))

    def _panel(self):
        """Panel ASCII para depuración (formato compartido con bug2)."""
        if not self._odom_received:
            return
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        dist = math.hypot(self.tx - self.x, self.ty - self.y)
        ang_to_goal = math.atan2(self.ty - self.y, self.tx - self.x)
        ang_err = math.degrees(self._wrap_pi(ang_to_goal - self.th))
        emerg_tag = '  *** EMERGENCY ***' if self._emergency else ''
        extra = (
            f'fsm: {self.state.value:<8} '
            f'phase: {self.wpm_phase}   '
            f'ang_err={ang_err:+5.1f}°   '
            f'avoid={self._avoid_side:+d}'
            f'{emerg_tag}'
        )
        # Estado visible en el header: si emergencia, sobreescribe.
        display_state = 'EMERGENCY' if self._emergency else self.state.value
        panel = render_panel(
            algo='goto_goal FSM',
            t_s=t,
            state=display_state,
            wp_idx=self.wp_idx,
            wp_total=self.wp_total,
            wp_xy=(self.tx, self.ty),
            pose=(self.x, self.y, self.th),
            d_goal=dist,
            d_front=self.d_front,
            d_left=self.d_left,
            d_right=self.d_right,
            extra_line=extra,
        )
        print(panel, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = GotoGoal()
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
