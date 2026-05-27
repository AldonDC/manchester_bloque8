"""
bug0 — Algoritmo Bug 0 simple y robusto
=========================================
Mini Challenge · Reactive Navigation · Week 6 · Task 1

Implementación del algoritmo Bug 0 de Choset, simplificado al máximo:

    GO_TO_GOAL:
        Apunta al goal. Si d_front < obstacle_threshold → WALL_FOLLOW
        registrando hit_dist_to_goal.

    WALL_FOLLOW:
        Bordea la pared del lado elegido con corrector P lateral.
        Salida: frente despejado (> clear_distance) Y progreso desde el
        HIT (>= min_hit_progress). Ambas condiciones son necesarias.

    GOAL_REACHED:
        dist < goal_tolerance. Terminal.

NO hay sub-fases, NO hay checks del LiDAR completo, NO hay histeresis.
El secreto es elegir los parámetros con base física correcta:

  - obstacle_threshold:  cuánto adelante "veo" antes de decidir bordear
  - clear_distance:      cuánto adelante despejado pido para salir
  - wall_distance:       a qué distancia mantengo la pared al costado
  - min_hit_progress:    cuánto debo haber avanzado al goal antes de salir
  - forward_step:        look-ahead del setpoint que doy al PID externo

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


class Bug0(Node):
    """Bug 0 minimalista: 3 estados, 5 parámetros físicos."""

    def __init__(self):
        super().__init__('bug0')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('control_period',      0.1)
        self.declare_parameter('target_x',            3.0)
        self.declare_parameter('target_y',            0.0)
        self.declare_parameter('goal_tolerance',      0.25)   # m
        self.declare_parameter('obstacle_threshold',  0.50)   # m, HIT
        self.declare_parameter('clear_distance',      0.90)   # m, libre para salir
        self.declare_parameter('wall_distance',       0.40)   # m, lateral deseada
        self.declare_parameter('kp_wall',             1.00)   # P lateral
        self.declare_parameter('forward_step',        0.80)   # look-ahead
        self.declare_parameter('min_hit_progress',    0.40)   # m progreso para salir

        self._load_params()

        # ── Estado ───────────────────────────────────────────────────────────
        self.state = State.GO_TO_GOAL
        self.x = self.y = self.th = 0.0
        self.tx = self.get_parameter('target_x').value
        self.ty = self.get_parameter('target_y').value
        self.d_front = self.d_left = self.d_right = float('inf')

        self.wall_side = +1
        self.hit_dist_to_goal = float('inf')

        # Pose registrada en el HIT (para medir distancia recorrida bordeando)
        self.hit_x = 0.0
        self.hit_y = 0.0

        # Flag: True cuando la pared ya está al COSTADO (no al frente).
        # En el HIT está al frente; hay que girar primero para ponerla
        # lateralmente antes de poder hacer wall-follow real.
        self.wall_engaged = False

        # ── Detección de loops (U-trap) ──────────────────────────────────
        self.loop_detect_radius = 0.60   # m, cercanía al HIT para detectar loop
        self.wall_follow_dist   = 0.0    # distancia acumulada bordeando
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.loop_flipped = False        # ya invertimos side por loop?

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
            f'Bug 0 | goal=({self.tx:.2f},{self.ty:.2f})  '
            f'obs_th={self.obstacle_threshold:.2f}  '
            f'clear={self.clear_distance:.2f}  '
            f'wall_d={self.wall_distance:.2f}  '
            f'hit_progress={self.min_hit_progress:.2f}'
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

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny, cosy)

    def _goal_cb(self, msg: Point):
        if abs(self.tx - msg.x) > 1e-3 or abs(self.ty - msg.y) > 1e-3:
            self.tx, self.ty = msg.x, msg.y
            self._set_state(State.GO_TO_GOAL, 'nuevo goal')

    def _df_cb(self, m: Float32): self.d_front = m.data
    def _dl_cb(self, m: Float32): self.d_left  = m.data
    def _dr_cb(self, m: Float32): self.d_right = m.data

    # ── FSM ──────────────────────────────────────────────────────────────────

    def _set_state(self, new_state: State, reason: str = ''):
        if new_state != self.state:
            self.get_logger().info(f'{self.state.value} → {new_state.value}  ({reason})')
            self.state = new_state

    def _loop(self):
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
        # Si lo que vemos enfrente está MÁS LEJOS que el goal (con margen),
        # no es un obstáculo en nuestro camino: ir directo.
        # Margen necesario: cuando d_front y dist_goal son parecidos
        # (caso típico al acercarse al goal tras bordear un obstáculo),
        # sin margen se dispara HIT contra el obstáculo que ya rebasamos.
        if self.d_front > dist_goal - 0.30:
            self._publish_setpoint(self.tx, self.ty)
            return

        # HIT: obstáculo enfrente más cerca que el goal.
        if self.d_front < self.obstacle_threshold:
            self.hit_dist_to_goal = dist_goal
            self.hit_x, self.hit_y = self.x, self.y
            self.wall_side = self._choose_wall_side()
            self.wall_engaged = False  # primero hay que girar
            self.wall_follow_dist = 0.0
            self.loop_flipped = False
            self.prev_x, self.prev_y = self.x, self.y
            self._set_state(
                State.WALL_FOLLOW,
                f'HIT  d_front={self.d_front:.2f}m  '
                f'd_L={self.d_left:.2f}  d_R={self.d_right:.2f}  '
                f'side={"L" if self.wall_side > 0 else "R"}  '
                f'dist_goal={dist_goal:.2f}m')
            return

        self._publish_setpoint(self.tx, self.ty)

    # ── Elección de lado para bordear ────────────────────────────────────────

    def _choose_wall_side(self) -> int:
        """
        Elige por qué lado bordear el obstáculo.

        Política: bordear por el lado con MÁS espacio libre.

        Verificación geométrica (yaw=0, robot mirando +X):
          `wall_side = +1`  →  enganche manda al robot al SUR (y < 0)
                            →  robot gira a su derecha
                            →  bordea el obstáculo por el SUR.
          `wall_side = -1`  →  enganche manda al robot al NORTE (y > 0)
                            →  robot gira a su izquierda
                            →  bordea por el NORTE.

        Por tanto, si hay MÁS espacio al sur del robot (d_right grande),
        queremos `wall_side = +1` para aprovecharlo. Si más espacio al
        norte (d_left grande), queremos `wall_side = -1`.

        Caso simétrico (|d_L - d_R| < 0.15): default a +1 (bordear por
        el sur), consistente con bug_easy.
        """
        if abs(self.d_left - self.d_right) < 0.15:
            return +1
        # d_right > d_left → más espacio al SUR → bordear por sur → +1.
        # d_left > d_right → más espacio al NORTE → bordear por norte → -1.
        return +1 if self.d_right > self.d_left else -1

    # ── WALL_FOLLOW ──────────────────────────────────────────────────────────

    def _do_wall_follow(self, dist_goal: float):
        progress = self.hit_dist_to_goal - dist_goal

        # ── Acumular distancia recorrida bordeando ───────────────────────────
        step = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        if step < 0.5:  # filtrar saltos de odometría
            self.wall_follow_dist += step
        self.prev_x, self.prev_y = self.x, self.y

        # ── DETECCIÓN DE LOOP (U-trap) ───────────────────────────────────────
        # Si después de recorrer un perímetro considerable el robot regresa
        # cerca del HIT original, significa que dio una vuelta completa al
        # obstáculo sin poder salir → invertimos wall_side para entrar por
        # el otro flanco de la U.
        dist_from_hit = math.hypot(self.x - self.hit_x, self.y - self.hit_y)
        if (self.wall_engaged and
                not self.loop_flipped and
                self.wall_follow_dist > 4.5 and
                dist_from_hit < self.loop_detect_radius):
            old_side = self.wall_side
            self.wall_side = -self.wall_side
            self.wall_engaged = False
            self.loop_flipped = True
            self.wall_follow_dist = 0.0
            self.hit_dist_to_goal = dist_goal
            self.get_logger().warn(
                f'LOOP detectado! Invertir side '
                f'{"L" if old_side > 0 else "R"} → '
                f'{"L" if self.wall_side > 0 else "R"}')
            return

        # ── Salida Bug 0: frente despejado + progreso real ───────────────────
        if (self.wall_engaged and
                self.d_front > self.clear_distance and
                progress >= self.min_hit_progress):
            self.wall_engaged = False
            self.loop_flipped = False
            self.wall_follow_dist = 0.0
            self._set_state(
                State.GO_TO_GOAL,
                f'libre + progreso={progress:.2f}m')
            self._publish_setpoint(self.tx, self.ty)
            return

        # Detecta cuándo la pared aparece al COSTADO (ya giramos lo suficiente).
        d_lateral_raw = self.d_left if self.wall_side > 0 else self.d_right
        if not self.wall_engaged and d_lateral_raw < 1.5 * self.wall_distance:
            self.wall_engaged = True
            self.get_logger().info(
                f'pared enganchada al lado {"L" if self.wall_side > 0 else "R"}, '
                f'd_lat={d_lateral_raw:.2f}m')

        # ── FASE 1: enganche (girar sin avanzar) ─────────────────────────────
        if not self.wall_engaged:
            cos_th, sin_th = math.cos(self.th), math.sin(self.th)
            opposite = -self.wall_side
            opp_x = -sin_th * opposite
            opp_y =  cos_th * opposite
            sp_x = self.x + 1.0 * opp_x
            sp_y = self.y + 1.0 * opp_y
            self._publish_setpoint(sp_x, sp_y)
            return

        # ── HIT SECUNDARIO: esquina interna ──────────────────────────────────
        # NO reseteamos hit_dist_to_goal: queremos medir progreso acumulado
        # desde el HIT original, no desde cada esquina. Así en oficinas con
        # múltiples paredes el robot puede salir al acumular suficiente
        # progreso real hacia el goal.
        if (self.wall_engaged and
                self.d_front < self.obstacle_threshold and
                dist_from_hit > 0.6):
            self.wall_engaged = False
            self.hit_x, self.hit_y = self.x, self.y
            self.get_logger().info(
                f'HIT secundario: esquina interna. '
                f'd_front={self.d_front:.2f}m  progress={progress:.2f}m')
            return

        # ── FASE 2: seguimiento de pared ─────────────────────────────────────
        d_lateral = min(d_lateral_raw, 2.0 * self.wall_distance)

        cos_th, sin_th = math.cos(self.th), math.sin(self.th)

        fwd = min(self.forward_step, dist_goal)

        if self.d_front < fwd + self.wall_distance:
            fwd = max(0.2, self.d_front - self.wall_distance)

        fx = self.x + fwd * cos_th
        fy = self.y + fwd * sin_th

        err = d_lateral - self.wall_distance
        bias = max(-0.5 * fwd, min(0.5 * fwd, self.kp_wall * err))
        side_x = -sin_th * self.wall_side
        side_y =  cos_th * self.wall_side
        sp_x = fx + bias * side_x
        sp_y = fy + bias * side_y

        self._publish_setpoint(sp_x, sp_y)

    # ── Publicación ──────────────────────────────────────────────────────────

    def _publish_setpoint(self, x: float, y: float):
        self.pub_setpoint.publish(Point(x=float(x), y=float(y), z=0.0))

    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.value))

    # ── Markers RViz ─────────────────────────────────────────────────────────

    def _markers(self):
        ma = MarkerArray()
        ma.markers.append(self._sphere('goal_final', 0, self.tx, self.ty, 0.20,
                                       ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)))
        ma.markers.append(self._line('robot_to_goal', 1,
                                     [(self.x, self.y), (self.tx, self.ty)],
                                     ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5), 0.02))
        ma.markers.append(self._disc('obs_radius', 2, self.x, self.y,
                                     self.obstacle_threshold * 2,
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
    node = Bug0()
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
