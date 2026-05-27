"""
bug2 — Algoritmo Bug 2 (Bug 0 + tracking de m-line)
=====================================================
Mini Challenge · Reactive Navigation · Week 6 · Task 2

Bug 2 es Bug 0 + una condición de salida más estricta de WALL_FOLLOW:

    Bug 0: sale cuando frente libre + progreso desde HIT
    Bug 2: sale cuando RECRUZAS la m-line (línea recta start→goal)
           más cerca del goal que el HIT anterior

La m-line es el segmento ideal de inicio a meta. Bug 2 promete:
"si me alejo de esta línea para bordear, no abandono el bordeo hasta
volver a tocarla y haber avanzado". Eso rompe el bucle infinito que
Bug 0 sufre en escenarios cóncavos (forma de U).

DIFERENCIA EN CÓDIGO vs bug0:
    · Guardamos (start_x, start_y) cuando recibimos la primera odometría
    · Función _distance_to_mline() implementa fórmula clásica de distancia
      punto-recta
    · Condición de salida en WALL_FOLLOW reemplazada:
          (frente libre Y progreso) → (frente libre Y en m-line Y progreso)
    · Visualización de la m-line en RViz (línea cyan)

EL RESTO ES IDÉNTICO A bug0.py: misma FSM, misma fase de enganche,
mismo wall-following P, mismo controller PID externo, mismos parámetros
físicos.

CONTRATO ROS:
    Subscribe:  odom, goal, bug/d_front, bug/d_left, bug/d_right
    Publica:    set_point, bug/state, bug/markers
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class State(str, Enum):
    GO_TO_GOAL   = 'GO_TO_GOAL'
    WALL_FOLLOW  = 'WALL_FOLLOW'
    GOAL_REACHED = 'GOAL_REACHED'


class Bug2(Node):
    """Bug 2: Bug 0 + m-line tracking. Robusto en escenarios cóncavos."""

    def __init__(self):
        super().__init__('bug2')

        # ── Parámetros (idénticos a bug0 + m-line) ───────────────────────────
        self.declare_parameter('control_period',      0.1)
        self.declare_parameter('target_x',            3.0)
        self.declare_parameter('target_y',            0.0)
        self.declare_parameter('goal_tolerance',      0.25)
        self.declare_parameter('obstacle_threshold',  0.50)
        self.declare_parameter('clear_distance',      0.90)
        self.declare_parameter('wall_distance',       0.40)
        self.declare_parameter('kp_wall',             1.00)
        self.declare_parameter('forward_step',        0.80)
        self.declare_parameter('min_hit_progress',    0.40)
        # ── Parámetros de m-line ─────────────────────────────────────────────
        # Tolerancia perpendicular a la m-line para considerar que la
        # estamos "tocando". Más generosa → sale antes (puede chocar);
        # más estricta → puede no salir nunca por ruido del odom.
        # 0.35m da margen para que un bordeo a 1.2m del eje X cuente como
        # "cerca de m-line" cuando rebasamos el obstáculo.
        self.declare_parameter('mline_tolerance',     0.35)

        self._load_params()

        # ── Estado ───────────────────────────────────────────────────────────
        self.state = State.GO_TO_GOAL
        self.x = self.y = self.th = 0.0
        self.tx = self.get_parameter('target_x').value
        self.ty = self.get_parameter('target_y').value
        self.d_front = self.d_left = self.d_right = float('inf')

        self.wall_side = +1
        self.hit_dist_to_goal = float('inf')
        self.wall_engaged = False

        # Pose registrada en el HIT
        self.hit_x = 0.0
        self.hit_y = 0.0

        # Distancia recorrida bordeando (para evitar salida prematura)
        self.wall_follow_dist = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        # Flag: true cuando ya nos alejamos de la m-line (evita salir
        # inmediatamente en el HIT que está SOBRE la m-line)
        self.left_mline = False

        # m-line: se fija con la primera lectura de odometría.
        # Si el goal cambia (via /goal), se recalcula.
        self.start_x = None
        self.start_y = None

        # ── ROS ──────────────────────────────────────────────────────────────
        self.create_subscription(Odometry, 'odom',       self._odom_cb, 10)
        self.create_subscription(Point,    'goal',       self._goal_cb, 10)
        self.create_subscription(Float32,  'bug/d_front', self._df_cb,  10)
        self.create_subscription(Float32,  'bug/d_left',  self._dl_cb,  10)
        self.create_subscription(Float32,  'bug/d_right', self._dr_cb,  10)

        self.pub_setpoint = self.create_publisher(Point,       'set_point',   10)
        self.pub_state    = self.create_publisher(String,      'bug/state',   10)
        self.pub_markers  = self.create_publisher(MarkerArray, 'bug/markers', 10)

        self.create_timer(self.dt, self._loop)
        self.create_timer(0.5, self._markers)

        self.get_logger().info(
            f'Bug 2 | goal=({self.tx:.2f},{self.ty:.2f})  '
            f'obs_th={self.obstacle_threshold:.2f}  '
            f'clear={self.clear_distance:.2f}  '
            f'wall_d={self.wall_distance:.2f}  '
            f'mline_tol={self.mline_tolerance:.2f}'
        )

    def _load_params(self):
        gp = lambda n: self.get_parameter(n).value
        self.dt                 = gp('control_period')
        self.goal_tolerance     = gp('goal_tolerance')
        self.obstacle_threshold = gp('obstacle_threshold')
        self.clear_distance     = gp('clear_distance')
        self.wall_distance      = gp('wall_distance')
        self.kp_wall            = gp('kp_wall')
        self.forward_step       = gp('forward_step')
        self.min_hit_progress   = gp('min_hit_progress')
        self.mline_tolerance    = gp('mline_tolerance')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny, cosy)
        # Fija la m-line en la primera lectura.
        if self.start_x is None:
            self.start_x, self.start_y = self.x, self.y
            self.get_logger().info(
                f'm-line fijada: ({self.start_x:.2f}, {self.start_y:.2f}) '
                f'→ ({self.tx:.2f}, {self.ty:.2f})')

    def _goal_cb(self, msg: Point):
        if abs(self.tx - msg.x) > 1e-3 or abs(self.ty - msg.y) > 1e-3:
            self.tx, self.ty = msg.x, msg.y
            # Nuevo goal → re-fijamos la m-line desde la pose actual.
            self.start_x, self.start_y = self.x, self.y
            self._set_state(State.GO_TO_GOAL, 'nuevo goal, m-line reseteada')

    def _df_cb(self, m: Float32): self.d_front = m.data
    def _dl_cb(self, m: Float32): self.d_left  = m.data
    def _dr_cb(self, m: Float32): self.d_right = m.data

    # ── FSM ──────────────────────────────────────────────────────────────────

    def _set_state(self, new_state: State, reason: str = ''):
        if new_state != self.state:
            self.get_logger().info(f'{self.state.value} → {new_state.value}  ({reason})')
            self.state = new_state

    def _loop(self):
        if self.start_x is None:
            return  # esperando primera odom

        dist_goal = math.hypot(self.tx - self.x, self.ty - self.y)

        if dist_goal < self.goal_tolerance:
            if self.state != State.GOAL_REACHED:
                self._set_state(State.GOAL_REACHED, f'dist={dist_goal:.2f}m')
            self._publish_state()
            return

        if self.state == State.GO_TO_GOAL:
            self._do_go_to_goal(dist_goal)
        else:
            self._do_wall_follow(dist_goal)

        self._publish_state()

    # ── GO_TO_GOAL ───────────────────────────────────────────────────────────

    def _do_go_to_goal(self, dist_goal: float):
        # Si lo que vemos enfrente está más lejos que el goal (con margen),
        # no nos bloquea: ir directo. Idéntico a bug0.
        if self.d_front > dist_goal - 0.30:
            self._publish_setpoint(self.tx, self.ty)
            return

        if self.d_front < self.obstacle_threshold:
            self.hit_dist_to_goal = dist_goal
            self.hit_x, self.hit_y = self.x, self.y
            self.wall_side = self._choose_wall_side()
            self.wall_engaged = False
            self.wall_follow_dist = 0.0
            self.prev_x, self.prev_y = self.x, self.y
            self.left_mline = False
            self._set_state(
                State.WALL_FOLLOW,
                f'HIT  d_front={self.d_front:.2f}m  '
                f'd_L={self.d_left:.2f}  d_R={self.d_right:.2f}  '
                f'side={"L" if self.wall_side > 0 else "R"}  '
                f'dist_goal={dist_goal:.2f}m')
            return

        self._publish_setpoint(self.tx, self.ty)

    def _choose_wall_side(self) -> int:
        """Idéntico a bug0: bordear por el lado con MÁS espacio libre."""
        if abs(self.d_left - self.d_right) < 0.15:
            return +1
        return +1 if self.d_right > self.d_left else -1

    # ── WALL_FOLLOW ──────────────────────────────────────────────────────────

    def _do_wall_follow(self, dist_goal: float):
        progress = self.hit_dist_to_goal - dist_goal
        d_mline = self._distance_to_mline(self.x, self.y)

        # ── Acumular distancia recorrida bordeando ───────────────────────────
        step = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        if step < 0.5:
            self.wall_follow_dist += step
        self.prev_x, self.prev_y = self.x, self.y

        # Detectar que nos ALEJAMOS de la m-line (para no salir en el HIT
        # cuando el HIT está justo sobre la m-line)
        if d_mline > self.mline_tolerance * 1.5:
            self.left_mline = True

        # ── OVERRIDE 1: PROXIMIDAD AL GOAL (ESTRICTO) ────────────────────────
        # Para salir de WALL_FOLLOW por proximidad exigimos TRES cosas:
        #   1. dist_goal < 0.6 m (muy cerca)
        #   2. goal está apuntando hacia adelante o casi (|ang_rel| < 60°)
        #   3. d_front > dist_goal + 0.1 m (CAMINO DIRECTO LIBRE)
        # Sin la (3), el robot sale cerca del goal pero con una pared en
        # medio y choca. Era el problema en bug_hard al bordear wall_top
        # por afuera.
        if dist_goal < 0.6:
            ang_world = math.atan2(self.ty - self.y, self.tx - self.x)
            ang_rel = math.atan2(math.sin(ang_world - self.th),
                                  math.cos(ang_world - self.th))
            goal_ahead = abs(math.degrees(ang_rel)) < 60.0
            front_clear = self.d_front > dist_goal + 0.10

            if goal_ahead and front_clear:
                self.wall_engaged = False
                self.left_mline = False
                self.wall_follow_dist = 0.0
                self._set_state(
                    State.GO_TO_GOAL,
                    f'goal proximity override  dist={dist_goal:.2f}m  '
                    f'd_front={self.d_front:.2f}m  ang_rel={math.degrees(ang_rel):.0f}°')
                self._publish_setpoint(self.tx, self.ty)
                return
            # Goal cerca pero bloqueado/atrás: seguir bordeando la pared.

        # ── OVERRIDE 2: WATCHDOG TOPOLÓGICO ──────────────────────────────────
        # Si llevamos > 4 m bordeando Y NO nos hemos acercado al goal
        # (dist_goal sigue ≥ hit_dist_to_goal − 0.5), estamos atrapados
        # por el lado equivocado del obstáculo. FLIPEAMOS el lado de
        # bordeo: en bug_hard esto saca al robot del callejón interno
        # y lo manda a bordear por afuera de la U.
        no_progress = (self.hit_dist_to_goal - dist_goal) < 0.5
        if self.wall_follow_dist > 4.0 and no_progress:
            self.wall_side = -self.wall_side
            self.wall_engaged = False
            self.left_mline = False
            self.wall_follow_dist = 0.0
            self.hit_x, self.hit_y = self.x, self.y
            self.hit_dist_to_goal = dist_goal
            self.get_logger().warn(
                f'WATCHDOG: 4m sin progreso, flip side → '
                f'{"L" if self.wall_side > 0 else "R"}  dist={dist_goal:.2f}m')
            return

        # ── SALIDA BUG 2 PURA (Cruzando m-line con progreso) ─────────────────
        # Salimos de la pared SOLO si cruzamos la m-line más cerca de la meta 
        # que el punto de impacto original.
        if (self.wall_engaged and
                self.d_front > self.obstacle_threshold and
                d_mline < self.mline_tolerance and
                progress > 0.10 and
                self.left_mline and
                self.wall_follow_dist > 1.0):
            
            self.wall_engaged = False
            self.left_mline = False
            self.wall_follow_dist = 0.0
            self._set_state(
                State.GO_TO_GOAL,
                f'm-line cruzada + progreso={progress:.2f}m')
            self._publish_setpoint(self.tx, self.ty)
            return

        # ── SEGUIMIENTO DE PARED (PID Continuo) ──────────────────────────────
        d_lateral_raw = self.d_left if self.wall_side > 0 else self.d_right
        
        # 1. ENGANCHE INICIAL
        if not self.wall_engaged and d_lateral_raw < 1.5 * self.wall_distance:
            self.wall_engaged = True
            self.get_logger().info(
                f'Pared enganchada (Lado {"Izquierdo" if self.wall_side > 0 else "Derecho"})'
            )

        if not self.wall_engaged:
            # Girar sobre el propio eje para buscar la pared
            cos_th, sin_th = math.cos(self.th), math.sin(self.th)
            opp = -self.wall_side
            self._publish_setpoint(self.x - sin_th * opp, self.y + cos_th * opp)
            return

        # 2. ESQUINA INTERNA: pared dobla hacia el robot.
        # En lugar de resetear el HIT (lo que falsifica wall_follow_dist y
        # rompe el watchdog), simplemente PEDIMOS un giro en sitio hacia
        # el lado opuesto de la pared. El controller PID hará el giro
        # sin avanzar. No reseteamos wall_engaged.
        if self.d_front < self.obstacle_threshold:
            cos_th, sin_th = math.cos(self.th), math.sin(self.th)
            opp = -self.wall_side
            # Setpoint perpendicular: gira en sitio hacia opuesto a la pared.
            sp_x = self.x - sin_th * opp * 0.5
            sp_y = self.y + cos_th * opp * 0.5
            self._publish_setpoint(sp_x, sp_y)
            return

        # 3. CONTROLADOR PID (Maneja rectas y esquinas externas fluidamente)
        d_lateral = min(d_lateral_raw, 2.0 * self.wall_distance)
        cos_th, sin_th = math.cos(self.th), math.sin(self.th)
        
        fwd = min(self.forward_step, dist_goal)
        # Suavizado de velocidad si hay algo enfrente (no necesariamente colisión inminente)
        if self.d_front < fwd + self.wall_distance:
            fwd = max(0.2, self.d_front - self.wall_distance)

        # Si perdimos la pared (esquina externa), reducimos la velocidad frontal 
        # y permitimos un giro más agresivo para abrazar la esquina.
        wall_lost = d_lateral_raw > 1.5 * self.wall_distance
        if wall_lost:
            fwd = 0.7  # Avanzamos despacito
            bias_limit = 2.0 * fwd  # Permitimos que el giro sea igual de fuerte que el avance (curva cerrada)
        else:
            bias_limit = 0.5 * fwd  # En recta, el giro es suave para no zigzaguear

        err = d_lateral - self.wall_distance
        
        # Limitamos la corrección lateral dinámicamente
        bias = max(-bias_limit, min(bias_limit, self.kp_wall * err))
        
        side_x = -sin_th * self.wall_side
        side_y =  cos_th * self.wall_side
        
        # Vector resultante: Avance + Corrección lateral
        sp_x = self.x + fwd * cos_th + bias * side_x
        sp_y = self.y + fwd * sin_th + bias * side_y
        self._publish_setpoint(sp_x, sp_y)

    # ── Visibilidad del goal: ¿hay camino directo? ───────────────────────────

    def _goal_visible(self, dist_goal: float) -> bool:
        """
        Devuelve True si el goal está en una dirección donde el LiDAR
        sectorizado reporta espacio libre suficiente.

        Calcula el ángulo robot→goal en frame del robot y elige el sector
        más cercano (front/left/right). Si ese sector reporta distancia
        >= dist_goal, el goal está visible.
        """
        # Ángulo al goal en frame del robot
        ang_world = math.atan2(self.ty - self.y, self.tx - self.x)
        ang_rel = math.atan2(math.sin(ang_world - self.th),
                              math.cos(ang_world - self.th))
        deg = math.degrees(ang_rel)

        # Front: ±30°, Left: 45°–135°, Right: -135°–-45°
        if -30 <= deg <= 30:
            sector_dist = self.d_front
        elif 45 <= deg <= 135:
            sector_dist = self.d_left
        elif -135 <= deg <= -45:
            sector_dist = self.d_right
        else:
            # Goal atrás del robot: no podemos verificar fácilmente, asumir bloqueado
            return False

        # Goal visible si el sector está al menos tan lejos como el goal
        # (margen de 0.20 m para considerar el goal libre de obstáculos)
        return sector_dist >= dist_goal - 0.20

    # ── m-line: distancia punto-recta clásica ────────────────────────────────

    def _distance_to_mline(self, x: float, y: float) -> float:
        """
        Distancia perpendicular del punto (x,y) a la recta que va de
        (start_x, start_y) a (tx, ty).

        Fórmula: |Δx_T·(y_s - y) − (x_s - x)·Δy_T| / |start→goal|
        """
        x1, y1 = self.start_x, self.start_y
        x2, y2 = self.tx, self.ty
        num = abs((x2 - x1) * (y1 - y) - (x1 - x) * (y2 - y1))
        den = math.hypot(x2 - x1, y2 - y1)
        return num / den if den > 1e-6 else float('inf')

    # ── Publicación ──────────────────────────────────────────────────────────

    def _publish_setpoint(self, x: float, y: float):
        self.pub_setpoint.publish(Point(x=float(x), y=float(y), z=0.0))

    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.value))

    # ── Markers RViz (con m-line visualizada) ────────────────────────────────

    def _markers(self):
        ma = MarkerArray()
        ma.markers.append(self._sphere('goal_final', 0, self.tx, self.ty, 0.20,
                                       ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)))
        # m-line en cyan: distintivo de Bug 2.
        if self.start_x is not None:
            ma.markers.append(self._line(
                'm_line', 1,
                [(self.start_x, self.start_y), (self.tx, self.ty)],
                ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8), 0.03))
        ma.markers.append(self._line(
            'robot_to_goal', 2,
            [(self.x, self.y), (self.tx, self.ty)],
            ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4), 0.02))
        ma.markers.append(self._disc(
            'obs_radius', 3, self.x, self.y, self.obstacle_threshold * 2,
            ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.12)))
        self.pub_markers.publish(ma)

    def _sphere(self, ns, mid, x, y, size, color):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.1
        m.scale.x = m.scale.y = m.scale.z = float(size)
        m.color = color
        return m

    def _disc(self, ns, mid, x, y, d, color):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.02
        m.scale.x = m.scale.y = float(d); m.scale.z = 0.01
        m.color = color
        return m

    def _line(self, ns, mid, points, color, w=0.02):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = float(w); m.color = color
        for px, py in points:
            m.points.append(Point(x=float(px), y=float(py), z=0.05))
        return m


def main(args=None):
    rclpy.init(args=args)
    node = Bug2()
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
