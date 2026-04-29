"""
Nodo: controller  (Parte 3 — Control PID de Posición)
=====================================================
Lazo de control cerrado que lleva al robot hacia un punto objetivo (x, y).

Responsabilidad:
    Calcular los comandos de velocidad lineal (v) y angular (ω) para que el
    Puzzlebot alcance una coordenada objetivo publicada por el generador de
    trayectorias. Al llegar al punto, notifica al generador para que avance
    al siguiente waypoint.

Estrategia de control:
    — Angular: PID completo (proporcional + integral + derivativo) sobre el
      error de orientación hacia el objetivo.
    — Lineal:  Proporcional puro sobre la distancia al objetivo.
    — Alineación primero: si el error angular supera 0.2 rad, se cancela la
      velocidad lineal para girar en sitio antes de avanzar. Esto produce
      esquinas más limpias en trayectorias geométricas.

Ecuaciones del controlador:
    error_dist  = sqrt((tx − x)² + (ty − y)²)
    error_ang   = atan2(ty − y, tx − x) − θ   [normalizado a (−π, π)]

    ω = kp_w · error_ang + ki_w · ∫error_ang·dt + kd_w · d(error_ang)/dt
    v = kv   · error_dist   (solo si |error_ang| ≤ 0.2 rad)

    Saturaciones: v ∈ [0, 0.3] m/s   |   ω ∈ [−1.5, 1.5] rad/s

Suscripciones:
    odom       (nav_msgs/Odometry)      — pose actual del robot
    set_point  (geometry_msgs/Point)    — objetivo recibido del generador

Publicaciones:
    cmd_vel    (geometry_msgs/Twist)    — comando de velocidad al simulador
    next_point (std_msgs/Bool)          — True cuando se alcanza el objetivo

Parámetros ROS 2:
    k_v          (default 0.5)    — ganancia proporcional lineal
    kp_w         (default 1.0)    — ganancia proporcional angular
    ki_w         (default 0.01)   — ganancia integral angular
    kd_w         (default 0.1)    — ganancia derivativa angular
    target_x     (default 0.0)    — objetivo X inicial [m]
    target_y     (default 0.0)    — objetivo Y inicial [m]
    dist_tolerance (default 0.05) — radio de aceptación del objetivo [m]
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool


class PositionController(Node):
    """Controlador PID de posición para el Puzzlebot."""

    def __init__(self):
        super().__init__('controller')

        # ── Parámetros PID ────────────────────────────────────────────────────
        self.declare_parameter('k_v',           0.5)    # ganancia lineal
        self.declare_parameter('kp_w',          1.0)    # proporcional angular
        self.declare_parameter('ki_w',          0.01)   # integral angular
        self.declare_parameter('kd_w',          0.1)    # derivativo angular
        self.declare_parameter('target_x',      0.0)    # objetivo X  [m]
        self.declare_parameter('target_y',      0.0)    # objetivo Y  [m]
        self.declare_parameter('dist_tolerance', 0.05)  # radio de aceptación [m]

        # ── Estado del PID angular ────────────────────────────────────────────
        self.integral_ang  = 0.0   # acumulador del término integral
        self.prev_ang_err  = 0.0   # error anterior para el término derivativo

        # ── Pose actual del robot (actualizada por odometría) ─────────────────
        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0

        # ── Bandera para enviar "llegué" solo una vez por objetivo ────────────
        # Evita spam de True en el tópico next_point cuando el robot se detiene.
        self.goal_reached_sent = False

        # ── Comunicación ROS 2 ────────────────────────────────────────────────
        self.sub_odom   = self.create_subscription(
            Odometry, 'odom',      self._odom_callback,   10)
        self.sub_target = self.create_subscription(
            Point,    'set_point', self._target_callback, 10)

        self.pub_cmd     = self.create_publisher(Twist, 'cmd_vel',    10)
        self.pub_reached = self.create_publisher(Bool,  'next_point', 10)

        # ── Timer del lazo de control a 50 Hz ─────────────────────────────────
        self.dt    = 0.02
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info('Controlador PID iniciado.')

    # ── Callbacks de suscripción ──────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        """Extrae la pose actual del mensaje de odometría."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convertir cuaternión a ángulo yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny, cosy)

    def _target_callback(self, msg: Point):
        """Actualiza el objetivo y resetea el estado del PID."""
        # Al recibir un nuevo punto se limpia el estado del integrador para evitar
        # que el error acumulado del objetivo anterior afecte al nuevo trayecto.
        self.integral_ang      = 0.0
        self.prev_ang_err      = 0.0
        self.goal_reached_sent = False   # habilitar la señal para este nuevo punto

        # Guardar nuevo objetivo como parámetros del nodo
        self.set_parameters([
            rclpy.parameter.Parameter('target_x', rclpy.Parameter.Type.DOUBLE, msg.x),
            rclpy.parameter.Parameter('target_y', rclpy.Parameter.Type.DOUBLE, msg.y),
        ])
        self.get_logger().info(f'Nuevo objetivo recibido: x={msg.x:.3f}, y={msg.y:.3f}')

    # ── Lazo de control ───────────────────────────────────────────────────────

    def _control_loop(self):
        """Calcula y publica el comando de velocidad en cada ciclo."""

        # Leer parámetros en cada ciclo (permiten ajuste en caliente)
        tx    = self.get_parameter('target_x').value
        ty    = self.get_parameter('target_y').value
        kv    = self.get_parameter('k_v').value
        kp_w  = self.get_parameter('kp_w').value
        ki_w  = self.get_parameter('ki_w').value
        kd_w  = self.get_parameter('kd_w').value
        tol   = self.get_parameter('dist_tolerance').value

        # 1. Calcular errores de posición y orientación
        dx   = tx - self.x
        dy   = ty - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        # Ángulo deseado y error angular (normalizado a (−π, π))
        ang_goal = math.atan2(dy, dx)
        ang_err  = math.atan2(math.sin(ang_goal - self.th),
                               math.cos(ang_goal - self.th))

        cmd     = Twist()
        reached = Bool(data=False)

        if dist > tol:
            # 2. PID angular
            self.integral_ang += ang_err * self.dt
            derivative         = (ang_err - self.prev_ang_err) / self.dt

            omega = (kp_w * ang_err
                     + ki_w * self.integral_ang
                     + kd_w * derivative)

            # 3. Velocidad lineal — estrategia "alinear primero, avanzar después"
            #    Si el error angular es grande se cancela v para girar en sitio.
            #    Umbral 0.2 rad (~11.5°) ofrece buen equilibrio entre velocidad
            #    y precisión en las esquinas de la trayectoria.
            if abs(ang_err) > 0.2:
                v = 0.0
            else:
                v = kv * dist

            # 4. Saturar comandos dentro de los límites del robot
            cmd.linear.x  = max(0.0, min(v,     0.3))
            cmd.angular.z = max(-1.5, min(omega, 1.5))

            self.prev_ang_err = ang_err

        else:
            # Objetivo alcanzado — detener el robot
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

            # Publicar True solo una vez por objetivo para no saturar el tópico
            if not self.goal_reached_sent:
                reached.data           = True
                self.goal_reached_sent = True
                self.get_logger().info(
                    f'Objetivo alcanzado: ({self.x:.3f}, {self.y:.3f})')

            # Resetear integral al llegar (evita windup en el siguiente tramo)
            self.integral_ang = 0.0

        self.pub_cmd.publish(cmd)
        self.pub_reached.publish(reached)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
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
