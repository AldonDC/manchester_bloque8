"""
bug1 — Algoritmo Bug 1 (vuelta completa al obstáculo)
=====================================================
Final Challenge · Part 2 · Reactive Navigation

Bug 1 es el algoritmo Bug clásico más robusto en términos de garantía
de llegada: si hay un camino, lo encuentra. A diferencia de Bug 0 y Bug
2 (reactivos, decisión local), Bug 1 hace exploración global del
contorno del obstáculo:

    1) GO_TO_GOAL: ir recto al goal hasta chocar.
    2) CIRCUMNAV_LAP: al chocar, registrar (hit_x, hit_y) y dar una
       VUELTA COMPLETA al obstáculo bordeándolo. Durante la vuelta
       memorizar el punto del contorno con la MENOR distancia al goal
       (closest_x, closest_y, closest_dist).
    3) GOTO_CLOSEST: seguir bordeando el mismo obstáculo hasta llegar
       al closest_point memorizado.
    4) Desde ahí → GO_TO_GOAL otra vez. Si vuelve a chocar antes de
       llegar, repetir todo el ciclo.

POR QUÉ ELEGÍ BUG 1 PARA PART 2 (vs Bug 0 / Bug 2):
    El laberinto de ekf_arena tiene CUARTOS INTERIORES (chambers iv1..
    iv4) que son obstáculos cóncavos casi cerrados. Si el robot
    arranca dentro de un chamber o entra a uno por error:
        · Bug 0 bordea las 4 paredes y nunca sale (no detecta abertura).
        · Bug 2 exige cruzar m-line con progreso; la m-line ATRAVIESA
          paredes sólidas, condición imposible → loop infinito.
        · Bug 1 SIEMPRE da una vuelta completa antes de decidir, así
          que aunque tarde encuentra la abertura del chamber y desde
          ahí dispara al goal.

    Costo: Bug 1 es más lento en mundos abiertos porque insiste en
    completar la vuelta aunque ya tenga camino libre. En laberintos
    cerrados ese costo se paga una vez y vale la pena.

CONTRATO ROS: idéntico a bug0 / bug2.
    Subscribe:  odom, goal, bug/d_front, bug/d_left, bug/d_right,
                waypoint_manager/state
    Publica:    set_point, bug/state, bug/markers
"""

import math
import re
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray

from puzzlebot_nav.navigation._status_panel import render as render_panel


class State(str, Enum):
    GO_TO_GOAL     = 'GO_TO_GOAL'
    CIRCUMNAV_LAP  = 'CIRCUMNAV_LAP'   # dando vuelta completa al obstáculo
    GOTO_CLOSEST   = 'GOTO_CLOSEST'    # yendo al closest_point por bordeo
    GOAL_REACHED   = 'GOAL_REACHED'


class Bug1(Node):
    """Bug 1: vuelta completa + closest_point + disparo final."""

    def __init__(self):
        super().__init__('bug1')

        # ── Parámetros (tuneados para laberinto ekf_arena) ───────────────────
        self.declare_parameter('control_period',      0.1)
        self.declare_parameter('target_x',            3.0)
        self.declare_parameter('target_y',            0.0)
        # Tolerancia para "llegué al waypoint". 0.40 m está dentro del
        # disco de la bandera (que tiene 0.36 m de diámetro) y bastante
        # generoso para que el waypoint cuente como tocado aunque la
        # odometría tenga 5-10 cm de drift acumulado. Probado: con 0.30 m
        # el robot pasaba a 0.32 m de la bandera y no la registraba.
        self.declare_parameter('goal_tolerance',      0.40)
        self.declare_parameter('obstacle_threshold',  0.40)
        # Distancia mínima al obstáculo más cercano POR DEBAJO de la cual
        # NO publicamos setpoint hacia adelante (el robot frena). Es el
        # cinturón de seguridad: si algo se cruza dentro de 0.25 m, lo
        # demás se ignora hasta que el LiDAR vuelva a ver libre.
        self.declare_parameter('safety_distance',     0.25)
        self.declare_parameter('clear_distance',      0.90)
        self.declare_parameter('wall_distance',       0.32)
        self.declare_parameter('kp_wall',             1.00)
        self.declare_parameter('forward_step',        0.80)
        # Vuelta completa cuenta solo si bordeé al menos esta distancia.
        # Evita falsos cierres por temblar cerca del hit_point. El perímetro
        # de un chamber del laberinto es ~3 m, así que 1.5 m es la mitad
        # mínima esperada antes de cerrar una vuelta.
        self.declare_parameter('min_lap_distance',    1.5)
        # Radio para considerar "volví al hit_point" o "llegué al closest".
        self.declare_parameter('return_tolerance',    0.30)
        # Early exit oportunista: durante CIRCUMNAV_LAP, si el robot ve goal
        # CERCA y con camino LIBRE adelante, abandona la vuelta y dispara.
        # 'early_exit_dist' = umbral de "cerca al goal" para activar el check.
        # 0.0 → desactivado (Bug 1 puro). 1.0 m → híbrido B1+B0 oportunista,
        # mucho más eficiente sin perder la garantía topológica (si nunca se
        # cumple la condición, completa la vuelta y va al closest como B1
        # clásico). Es lo que la literatura llama "Bug 1 with shortcut".
        self.declare_parameter('early_exit_dist',     1.5)

        self._load_params()

        # ── Estado ───────────────────────────────────────────────────────────
        self.state = State.GO_TO_GOAL
        self.x = self.y = self.th = 0.0
        self.tx = self.get_parameter('target_x').value
        self.ty = self.get_parameter('target_y').value
        self.d_front = self.d_left = self.d_right = float('inf')

        self._odom_received = False
        self._lidar_ready = False
        self._goal_received = False

        # Hit point: donde chocamos esta vez con el obstáculo.
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.hit_dist_to_goal = float('inf')

        # Closest point: el punto del contorno más cercano al goal visto
        # durante la vuelta. Bug 1 va hacia este punto antes de disparar.
        self.closest_x = 0.0
        self.closest_y = 0.0
        self.closest_dist = float('inf')

        # Distancia acumulada bordeando esta vuelta (para min_lap_distance
        # y para detectar el cierre real, no el tembleque inicial).
        self.wall_follow_dist = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0

        # Lado de bordeo (+1 = pared a la izquierda, -1 = derecha).
        self.wall_side = +1
        self.wall_engaged = False

        # Para el panel de status (idéntico al de bug2).
        self.wp_idx = 0
        self.wp_total = 4
        self.wpm_phase = '...'

        # ── ROS plumbing (mismo contrato que bug0/bug2) ───────────────────────
        self.pub_setpoint = self.create_publisher(Point, 'set_point', 10)
        self.pub_state = self.create_publisher(String, 'bug/state', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, 'bug/markers', 10)

        self.create_subscription(Odometry, 'odom',       self._odom_cb, 10)
        self.create_subscription(Point,    'goal',       self._goal_cb, 10)
        self.create_subscription(Float32,  'bug/d_front', self._df_cb,  10)
        self.create_subscription(Float32,  'bug/d_left',  self._dl_cb,  10)
        self.create_subscription(Float32,  'bug/d_right', self._dr_cb,  10)
        self.create_subscription(String, 'waypoint_manager/state',
                                 self._wpm_cb, 10)

        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(self.dt, self._loop)
        # Panel ASCII en terminal cada 0.5 s.
        self.create_timer(0.5, self._panel)

        self.get_logger().info(
            f'Bug 1 | goal=({self.tx:.2f},{self.ty:.2f})  '
            f'tol={self.goal_tolerance:.2f}m'
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
        self.min_lap_distance   = gp('min_lap_distance')
        self.return_tolerance   = gp('return_tolerance')
        self.early_exit_dist    = gp('early_exit_dist')
        self.safety_distance    = gp('safety_distance')

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
            self.prev_x, self.prev_y = self.x, self.y
            self.get_logger().info(
                f'Primera odometría: pose=({self.x:.2f},{self.y:.2f})'
                f' → goal=({self.tx:.2f},{self.ty:.2f})')

    def _goal_cb(self, msg: Point):
        if abs(self.tx - msg.x) > 1e-3 or abs(self.ty - msg.y) > 1e-3:
            self.tx, self.ty = msg.x, msg.y
            # Reset Bug 1 state al cambiar de waypoint: nueva vuelta limpia.
            self.state = State.GO_TO_GOAL
            self.wall_engaged = False
            self.wall_follow_dist = 0.0
            self.closest_dist = float('inf')
            self.get_logger().info(
                f'Nuevo goal: ({self.tx:.2f},{self.ty:.2f}) → reset Bug 1')
        self._goal_received = True

    def _df_cb(self, m: Float32):
        if not self._lidar_ready and math.isfinite(m.data):
            self._lidar_ready = True
        self.d_front = m.data

    def _dl_cb(self, m: Float32): self.d_left  = m.data
    def _dr_cb(self, m: Float32): self.d_right = m.data

    def _wpm_cb(self, msg: String):
        # Parsear idx para el panel (formato: "STATE=... idx=N goal=...").
        m = re.search(r'idx=(\d+)', msg.data or '')
        if m:
            self.wp_idx = int(m.group(1))
        self.wpm_phase = msg.data.split()[0] if msg.data else '...'
        # Si el waypoint manager está en DWELL, dejamos al controller
        # quieto publicando set_point en pose actual.
        if 'DWELL' in msg.data or 'FINISHED' in msg.data:
            self._publish_setpoint(self.x, self.y)

    def _set_state(self, new_state: State, reason: str = ''):
        if new_state != self.state:
            self.get_logger().info(
                f'STATE: {self.state.value} → {new_state.value}  {reason}')
            self.state = new_state

    # ── Loop principal ───────────────────────────────────────────────────────

    def _loop(self):
        if (not self._odom_received
                or not self._lidar_ready
                or not self._goal_received):
            return

        dist_goal = math.hypot(self.tx - self.x, self.ty - self.y)

        if dist_goal < self.goal_tolerance:
            if self.state != State.GOAL_REACHED:
                self._set_state(State.GOAL_REACHED, f'dist={dist_goal:.2f}m')
            self._publish_state()
            return

        if self.state == State.GO_TO_GOAL:
            self._do_go_to_goal(dist_goal)
        elif self.state == State.CIRCUMNAV_LAP:
            self._do_circumnav(dist_goal)
        elif self.state == State.GOTO_CLOSEST:
            self._do_goto_closest(dist_goal)

        self._publish_state()

    # ── GO_TO_GOAL ───────────────────────────────────────────────────────────

    def _do_go_to_goal(self, dist_goal: float):
        # Camino libre adelante (con margen) → ir directo.
        if self.d_front > dist_goal - 0.30:
            self._publish_setpoint(self.tx, self.ty)
            return

        # Si vemos algo adentro del umbral, registramos HIT y arrancamos
        # la vuelta completa Bug 1.
        if self.d_front < self.obstacle_threshold:
            self.hit_x, self.hit_y = self.x, self.y
            self.hit_dist_to_goal = dist_goal
            self.wall_side = self._choose_wall_side()
            self.wall_engaged = False
            self.wall_follow_dist = 0.0
            # Inicializar el closest_point en el hit (siempre dentro del lap).
            self.closest_x, self.closest_y = self.x, self.y
            self.closest_dist = dist_goal
            self.prev_x, self.prev_y = self.x, self.y
            self._set_state(
                State.CIRCUMNAV_LAP,
                f'HIT  d_front={self.d_front:.2f}m  '
                f'dist_goal={dist_goal:.2f}m  '
                f'side={"L" if self.wall_side > 0 else "R"}  '
                f'→ vuelta completa')
            return

        # Frente nominalmente libre pero no claramente libre → seguir recto.
        self._publish_setpoint(self.tx, self.ty)

    def _choose_wall_side(self) -> int:
        """Bordear por el lado con MÁS espacio libre (igual que bug0/bug2)."""
        if abs(self.d_left - self.d_right) < 0.15:
            return +1
        return +1 if self.d_right > self.d_left else -1

    def _goal_clear_in_sector(self, dist_goal: float) -> bool:
        """¿Hay camino libre al goal usando los 3 sectores del LiDAR?

        El bug original chequeaba solo d_front, pero durante un bordeo el
        robot va MIRANDO la pared (no el goal), así que d_front nunca
        confirma. Aquí calculamos en qué sector cae el ángulo robot→goal y
        consultamos la distancia de ESE sector. Esto es lo que permite que
        el shortcut funcione aunque el robot esté de costado al goal.

        Sectores publicados por lidar_processor (relativos al frente
        del robot = ángulo 0):
            front:  ±30°
            left :  +90° ±45°   (es decir, +45° a +135°)
            right:  -90° ±45°   (es decir, -135° a -45°)
        Lo que queda (|ang| > 135°) es la parte trasera y NO hay sector.
        Para garantizar seguridad, si el goal queda detrás devolvemos
        False (no podemos confirmar camino libre con la información
        disponible).
        """
        ang_world = math.atan2(self.ty - self.y, self.tx - self.x)
        ang_rel = math.atan2(math.sin(ang_world - self.th),
                              math.cos(ang_world - self.th))
        deg = math.degrees(ang_rel)

        if -30.0 <= deg <= 30.0:
            sector_dist = self.d_front
        elif 45.0 <= deg <= 135.0:
            sector_dist = self.d_left
        elif -135.0 <= deg <= -45.0:
            sector_dist = self.d_right
        else:
            # Ángulo muerto entre sectores (30°-45°, etc.) o trasero:
            # no podemos confirmar; mejor no disparar.
            return False

        # Margen 0.10 m: exigimos al menos 10 cm más allá del goal libres
        # para tener seguridad de paso.
        return sector_dist > dist_goal + 0.10

    # ── CIRCUMNAV_LAP: vuelta completa registrando closest ───────────────────

    def _do_circumnav(self, dist_goal: float):
        """Bordea el obstáculo. Si encontramos punto más cercano al goal,
        lo memorizamos. Si volvemos al hit_point con suficiente vuelta
        recorrida, pasamos a GOTO_CLOSEST.

        Early exit oportunista: si durante la vuelta el robot pasa CERCA
        del goal con camino LIBRE, no insiste en completar la vuelta —
        dispara directo. Mantiene la garantía topológica de Bug 1 porque
        si la condición nunca se cumple, sigue siendo Bug 1 clásico.
        """
        # 1. Acumular distancia recorrida (con clamp anti-glitch de odom).
        step = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        if step < 0.5:
            self.wall_follow_dist += step
        self.prev_x, self.prev_y = self.x, self.y

        # 2. Actualizar closest_point si encontramos uno mejor.
        if dist_goal < self.closest_dist:
            self.closest_dist = dist_goal
            self.closest_x, self.closest_y = self.x, self.y

        # 2.5. EARLY EXIT OPORTUNISTA — la mejora vs Bug 1 clásico.
        # Bug 1 puro NO lo hace; mantenemos su garantía topológica porque
        # si esta condición nunca se cumple el algoritmo completa la vuelta
        # y va al closest_point (rama original abajo).
        if (self.wall_follow_dist > 0.5
                and self.early_exit_dist > 0.0
                and dist_goal < self.early_exit_dist
                and self._goal_clear_in_sector(dist_goal)):
            self.wall_engaged = False
            self._set_state(
                State.GO_TO_GOAL,
                f'EARLY EXIT (shortcut)  dist_goal={dist_goal:.2f}m  '
                f'wf={self.wall_follow_dist:.2f}m')
            self._publish_setpoint(self.tx, self.ty)
            return

        # 3. ¿Ya cerré la vuelta completa? Volví al hit_point Y la
        #    distancia bordeada es razonable (no es el tembleque inicial).
        d_to_hit = math.hypot(self.x - self.hit_x, self.y - self.hit_y)
        if (self.wall_follow_dist > self.min_lap_distance
                and d_to_hit < self.return_tolerance):
            # Vuelta completa terminada. Si el closest_point ES el hit
            # (nadie mejoró), el closest es donde estamos ahora.
            self._set_state(
                State.GOTO_CLOSEST,
                f'vuelta completa: {self.wall_follow_dist:.1f}m  '
                f'closest=({self.closest_x:.2f},{self.closest_y:.2f}) '
                f'dist_closest={self.closest_dist:.2f}m')
            return

        # 4. Si no, seguir bordeando con el mismo controller del Bug 2.
        self._wall_follow_step(dist_goal)

    # ── GOTO_CLOSEST: ir al closest_point por el mismo bordeo ────────────────

    def _do_goto_closest(self, dist_goal: float):
        """Sigue bordeando hasta llegar al closest_point. Al llegar,
        suelta el muro y vuelve a GO_TO_GOAL para disparar al goal.
        Igual que en CIRCUMNAV_LAP, si en el camino al closest detectamos
        camino libre al goal, disparamos antes (early exit).
        """
        # Early exit oportunista (mismo criterio que CIRCUMNAV_LAP).
        if (self.early_exit_dist > 0.0
                and dist_goal < self.early_exit_dist
                and self._goal_clear_in_sector(dist_goal)):
            self.wall_engaged = False
            self._set_state(
                State.GO_TO_GOAL,
                f'EARLY EXIT (en GOTO_CLOSEST)  dist_goal={dist_goal:.2f}m')
            self._publish_setpoint(self.tx, self.ty)
            return

        d_to_closest = math.hypot(
            self.x - self.closest_x, self.y - self.closest_y)
        if d_to_closest < self.return_tolerance:
            # ¡Llegamos al closest_point! Soltar el muro y disparar.
            self.wall_engaged = False
            self._set_state(
                State.GO_TO_GOAL,
                f'closest alcanzado: dist_goal={dist_goal:.2f}m  '
                f'→ disparo final')
            self._publish_setpoint(self.tx, self.ty)
            return

        # Seguir bordeando hacia el closest.
        self._wall_follow_step(dist_goal)

    # ── Wall-follow controller (idéntico a bug2) ─────────────────────────────

    def _wall_follow_step(self, dist_goal: float):
        d_lateral_raw = self.d_left if self.wall_side > 0 else self.d_right

        # 1. ENGANCHE INICIAL
        if not self.wall_engaged and d_lateral_raw < 1.5 * self.wall_distance:
            self.wall_engaged = True

        if not self.wall_engaged:
            # Girar sobre el propio eje para buscar la pared.
            cos_th, sin_th = math.cos(self.th), math.sin(self.th)
            opp = -self.wall_side
            self._publish_setpoint(self.x - sin_th * opp, self.y + cos_th * opp)
            return

        # 2. ESQUINA INTERNA: pared dobla hacia el robot.
        if self.d_front < self.obstacle_threshold:
            cos_th, sin_th = math.cos(self.th), math.sin(self.th)
            opp = -self.wall_side
            sp_x = self.x - sin_th * opp * 0.5
            sp_y = self.y + cos_th * opp * 0.5
            self._publish_setpoint(sp_x, sp_y)
            return

        # 3. CONTROLADOR P de seguimiento de pared.
        d_lateral = min(d_lateral_raw, 2.0 * self.wall_distance)
        cos_th, sin_th = math.cos(self.th), math.sin(self.th)

        fwd = min(self.forward_step, dist_goal)
        if self.d_front < fwd + self.wall_distance:
            fwd = max(0.2, self.d_front - self.wall_distance)

        wall_lost = d_lateral_raw > 1.5 * self.wall_distance
        if wall_lost:
            fwd = 0.7
            bias_limit = 2.0 * fwd
        else:
            bias_limit = 0.5 * fwd

        err = d_lateral - self.wall_distance
        bias = max(-bias_limit, min(bias_limit, self.kp_wall * err))

        side_x = -sin_th * self.wall_side
        side_y =  cos_th * self.wall_side

        sp_x = self.x + fwd * cos_th + bias * side_x
        sp_y = self.y + fwd * sin_th + bias * side_y
        self._publish_setpoint(sp_x, sp_y)

    # ── Publicación ──────────────────────────────────────────────────────────

    def _publish_setpoint(self, x: float, y: float):
        """Publica un setpoint con cinturón de seguridad anti-choque.

        Verifica el sector LiDAR en el que cae el rumbo robot→setpoint:
        si reporta distancia < safety_distance, NO publicamos el setpoint
        hacia adelante: en su lugar mandamos al robot a frenar en su pose
        actual. Esto evita que un setpoint "agresivo" (early exit, wall
        follow demasiado lateral, etc.) lance al robot contra un muro
        cercano. El controller verá un setpoint = pose actual y reducirá
        velocidad a cero hasta que el LiDAR vuelva a ver libre.
        """
        ang_world = math.atan2(y - self.y, x - self.x)
        ang_rel = math.atan2(math.sin(ang_world - self.th),
                              math.cos(ang_world - self.th))
        deg = math.degrees(ang_rel)
        if -30.0 <= deg <= 30.0:
            sector_dist = self.d_front
        elif 45.0 <= deg <= 135.0:
            sector_dist = self.d_left
        elif -135.0 <= deg <= -45.0:
            sector_dist = self.d_right
        else:
            # Rumbo en ángulo muerto (lados 30°-45°) o trasero: no hay
            # información, dejamos pasar (peor caso lo maneja el LiDAR
            # del controller).
            sector_dist = float('inf')

        if sector_dist < self.safety_distance:
            # FRENO: el sector está bloqueado, no avanzar. Setpoint =
            # pose actual hace que el controller saque v_lineal ~0.
            self.pub_setpoint.publish(
                Point(x=float(self.x), y=float(self.y), z=0.0))
            return

        self.pub_setpoint.publish(Point(x=float(x), y=float(y), z=0.0))

    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.value))

    def _panel(self):
        """Panel ASCII con estado actual (mismo formato que bug2)."""
        if not self._odom_received:
            return
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        dist_goal = math.hypot(self.tx - self.x, self.ty - self.y)
        d_closest = math.hypot(
            self.x - self.closest_x, self.y - self.closest_y)
        extra = (
            f'phase: {self.wpm_phase}   '
            f'wf_dist={self.wall_follow_dist:.2f} m   '
            f'closest_dist={self.closest_dist:.2f} m   '
            f'd_closest={d_closest:.2f} m'
        )
        panel = render_panel(
            algo='Bug 1',
            t_s=t,
            state=self.state.value,
            wp_idx=self.wp_idx,
            wp_total=self.wp_total,
            wp_xy=(self.tx, self.ty),
            pose=(self.x, self.y, self.th),
            d_goal=dist_goal,
            d_front=self.d_front,
            d_left=self.d_left,
            d_right=self.d_right,
            extra_line=extra,
        )
        print(panel, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = Bug1()
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
