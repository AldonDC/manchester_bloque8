"""
waypoint_manager — Closed-loop multi-waypoint sequencer for Part 2.

Reads a list of waypoints from parameters, publishes one at a time to /goal,
and advances to the next when the Bug node reports 'GOAL_REACHED' on
/bug/state. After the last waypoint, it loops back to the first to close
the trajectory, satisfying the Final Challenge requirement of a closed
trajectory with >=4 target points.

Contract:
    Publishes:  /goal                       (geometry_msgs/Point)
                /visualization_marker_array (visualization_msgs/MarkerArray)
                /waypoint_manager/state     (std_msgs/String)
    Subscribes: /bug/state                  (std_msgs/String)

Parameters:
    waypoints       flat list [x0,y0, x1,y1, ...] in metres (>= 8 entries -> 4 pts)
    frame_id        marker frame                                  (default 'odom')
    loop            close the trajectory by cycling back to wp 0  (default True)
    dwell_time      seconds to wait after reaching a wp           (default 1.0)
    publish_period  re-publish current goal every N s             (default 1.0)

Why a dedicated manager (not just chaining inside Bug):
    Keeps the Bug nodes single-goal, which is how the Mini Challenge had them.
    The manager is a thin sequencer above Bug, decoupled and testable on its
    own. Same Bug binary handles one or many waypoints.
"""

from enum import Enum

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class _State(Enum):
    RUNNING = 'RUNNING'
    DWELL = 'DWELL'
    FINISHED = 'FINISHED'


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('waypoints',
                               [2.0, 0.0,
                                2.0, 2.0,
                                0.0, 2.0,
                                -1.5, 1.0])
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('loop', True)
        self.declare_parameter('dwell_time', 1.0)
        self.declare_parameter('publish_period', 1.0)
        # Delay inicial antes de publicar el primer /goal. Durante esos
        # segundos las banderas y la ruta ya se ven en RViz y Gazebo, pero
        # el robot no recibe goal (luego está quieto). Sirve para que el
        # stack (Gazebo + camera + EKF + bridge) esté plenamente arriba
        # antes de empezar a moverse.
        self.declare_parameter('startup_delay', 5.0)
        # Watchdog timeout: si en max_wp_time el nav no publica GOAL_REACHED,
        # forzamos avance al siguiente waypoint. Evita misión bloqueada por
        # un wp inalcanzable. 45s es generoso para navegación reactiva.
        self.declare_parameter('max_wp_time', 45.0)

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) < 8 or len(flat) % 2 != 0:
            raise ValueError(
                f'waypoints must be a flat list of >=8 even-length floats '
                f'(>=4 waypoints). Got {len(flat)} entries.'
            )
        self.waypoints = [(float(flat[i]), float(flat[i + 1]))
                          for i in range(0, len(flat), 2)]

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.loop = bool(self.get_parameter('loop').value)
        self.dwell_time = float(self.get_parameter('dwell_time').value)
        period = float(self.get_parameter('publish_period').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.max_wp_time = float(self.get_parameter('max_wp_time').value)

        self.idx = 0
        self.state = _State.RUNNING
        self.dwell_started_at = None
        # Reloj de arranque para el startup delay.
        self._t0 = self.get_clock().now()
        self._startup_logged = False
        # Reloj del waypoint actual (para el watchdog max_wp_time).
        self.wp_started_at = self._t0

        self.pub_goal = self.create_publisher(Point, 'goal', 10)
        self.pub_state = self.create_publisher(String, 'waypoint_manager/state', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, 'visualization_marker_array', 10)

        self.create_subscription(String, 'bug/state', self._bug_state_cb, 10)
        # Suscripción a odom para detección de llegada INDEPENDIENTE del bug.
        # Práctica robótica senior: el waypoint_manager NO debe depender de
        # que el navegador publique correctamente GOAL_REACHED — si el bug
        # falla en publicar, la misión se bloquea. Con su propia detección
        # por distancia, la avance garantizado.
        self._robot_x = None
        self._robot_y = None
        self.declare_parameter('arrival_radius', 0.50)  # un poco más generoso que el bug (0.45)
        self.arrival_radius = float(self.get_parameter('arrival_radius').value)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.timer = self.create_timer(period, self._tick)

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

        self.get_logger().debug(
            f'WaypointManager | {len(self.waypoints)} waypoints, '
            f'loop={self.loop}, dwell={self.dwell_time:.1f}s'
        )

    def _bug_state_cb(self, msg: String):
        if self.state != _State.RUNNING:
            return
        if msg.data == 'GOAL_REACHED':
            self.get_logger().info(
                f'wp[{self.idx}] reached @ {self.waypoints[self.idx]}'
            )
            self.state = _State.DWELL
            self.dwell_started_at = self.get_clock().now()

    def _advance(self):
        self.idx += 1
        if self.idx >= len(self.waypoints):
            if self.loop:
                self.idx = 0
                self.get_logger().info('closing trajectory — back to wp[0]')
            else:
                self.state = _State.FINISHED
                self.get_logger().info('trajectory complete (loop=False)')
                return
        self.state = _State.RUNNING
        # Reset del reloj watchdog para el nuevo waypoint.
        self.wp_started_at = self.get_clock().now()
        self.get_logger().info(
            f'→ persiguiendo wp[{self.idx}] = {self.waypoints[self.idx]}')

    def _tick(self):
        now = self.get_clock().now()

        if self.state == _State.DWELL:
            elapsed = (now - self.dwell_started_at).nanoseconds * 1e-9
            if elapsed >= self.dwell_time:
                self._advance()

        # PROXIMITY DETECTION: arrival INDEPENDIENTE del bug.
        # Si el robot está dentro del arrival_radius del waypoint actual,
        # avanzamos directo. No esperamos a que el bug publique GOAL_REACHED
        # — esto resuelve el bug donde el bug entra a estado ARRIVED pero
        # por alguna razón no publica el string esperado, y la misión se
        # queda bloqueada con el robot encima del waypoint.
        if (self.state == _State.RUNNING
                and self._robot_x is not None
                and self._robot_y is not None):
            wx, wy = self.waypoints[self.idx]
            d = math.hypot(self._robot_x - wx, self._robot_y - wy)
            if d < self.arrival_radius:
                self.get_logger().info(
                    f'wp[{self.idx}] reached @ {self.waypoints[self.idx]} '
                    f'(proximity d={d:.2f}m)')
                self.state = _State.DWELL
                self.dwell_started_at = now

        # WATCHDOG: si llevamos > max_wp_time segundos persiguiendo el mismo
        # waypoint sin alcanzarlo, asumimos que está atorado y FORZAMOS
        # el avance. Evita misión bloqueada por wp inalcanzable. Práctica
        # estándar de robótica de campo.
        if self.state == _State.RUNNING:
            elapsed_wp = (now - self.wp_started_at).nanoseconds * 1e-9
            if elapsed_wp >= self.max_wp_time:
                self.get_logger().warn(
                    f'WATCHDOG: wp[{self.idx}] no alcanzado en '
                    f'{elapsed_wp:.0f}s — FORZANDO avance al siguiente')
                self.state = _State.DWELL
                self.dwell_started_at = now

        self._publish_current_goal()
        self._publish_markers()
        self._publish_status()

    def _publish_current_goal(self):
        if self.state == _State.FINISHED:
            return
        # Startup delay: durante los primeros N segundos NO publicamos el
        # goal todavía. Las banderas/ruta sí se ven (los markers se
        # publican igual desde _tick → _publish_markers), pero el Bug
        # se queda quieto porque no tiene goal.
        elapsed = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        if elapsed < self.startup_delay:
            if not self._startup_logged:
                self.get_logger().info(
                    f'Waiting {self.startup_delay:.1f}s before publishing '
                    f'first goal (stack warmup)…')
                self._startup_logged = True
            return
        x, y = self.waypoints[self.idx]
        self.pub_goal.publish(Point(x=x, y=y, z=0.0))

    def _publish_status(self):
        tag = f'{self.state.value} wp[{self.idx}]'
        self.pub_state.publish(String(data=tag))

    def _publish_markers(self):
        """Publica banderas estilo Gazebo en RViz para cada waypoint.

        Cada bandera = disco verde en el suelo + poste blanco + esfera
        verde grande en la cima + label "WP0..3" flotante. El waypoint
        activo se pinta naranja brillante y un poco más grande para que
        sea evidente cuál está sirviendo el manager ahora.
        """
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        GREEN = (0.10, 0.85, 0.20, 1.0)
        GREEN_DIM = (0.10, 0.65, 0.20, 0.85)
        ORANGE = (1.00, 0.55, 0.05, 1.0)

        for i, (x, y) in enumerate(self.waypoints):
            active = (i == self.idx and self.state != _State.FINISHED)
            color = ORANGE if active else GREEN if i == 0 else GREEN_DIM

            # 1) Disco en el suelo (marca el punto exacto del goal).
            disc = Marker()
            disc.header.frame_id = self.frame_id
            disc.header.stamp = now
            disc.ns = 'wp_disc'
            disc.id = i
            disc.type = Marker.CYLINDER
            disc.action = Marker.ADD
            disc.pose.position.x = x
            disc.pose.position.y = y
            disc.pose.position.z = 0.005
            disc.scale.x = disc.scale.y = 0.36
            disc.scale.z = 0.01
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = color
            ma.markers.append(disc)

            # 2) Poste blanco delgado.
            post = Marker()
            post.header.frame_id = self.frame_id
            post.header.stamp = now
            post.ns = 'wp_post'
            post.id = i
            post.type = Marker.CYLINDER
            post.action = Marker.ADD
            post.pose.position.x = x
            post.pose.position.y = y
            post.pose.position.z = 0.25
            post.scale.x = post.scale.y = 0.025
            post.scale.z = 0.50
            post.color.r, post.color.g, post.color.b, post.color.a = (
                0.95, 0.95, 0.95, 1.0)
            ma.markers.append(post)

            # 3) Esfera grande arriba del poste (la "bandera").
            ball = Marker()
            ball.header.frame_id = self.frame_id
            ball.header.stamp = now
            ball.ns = 'wp_ball'
            ball.id = i
            ball.type = Marker.SPHERE
            ball.action = Marker.ADD
            ball.pose.position.x = x
            ball.pose.position.y = y
            ball.pose.position.z = 0.55
            r = 0.20 if active else 0.16
            ball.scale.x = ball.scale.y = ball.scale.z = r
            ball.color.r, ball.color.g, ball.color.b, ball.color.a = color
            ma.markers.append(ball)

            # 4) Label flotante "WP0", "WP1", etc.
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = 'wp_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.85
            label.scale.z = 0.20
            label.color.r, label.color.g, label.color.b, label.color.a = (
                1.0, 1.0, 1.0, 1.0)
            label.text = f'WP{i}' + (' ◄ active' if active else '')
            ma.markers.append(label)

        # 5) Línea que une los waypoints en orden, para ver la ruta cerrada.
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = 'wp_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = (
            0.10, 0.85, 0.20, 0.55)
        pts = list(self.waypoints) + ([self.waypoints[0]] if self.loop else [])
        for x, y in pts:
            p = Point()
            p.x, p.y, p.z = x, y, 0.04
            line.points.append(p)
        ma.markers.append(line)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
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
