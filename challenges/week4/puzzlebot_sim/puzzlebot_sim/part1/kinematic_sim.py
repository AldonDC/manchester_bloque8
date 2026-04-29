"""
Nodo: kinematic_sim  (Parte 1 — Simulación Cinemática)
=======================================================
Representa la "planta física" virtual del Puzzlebot.

Responsabilidad:
    Integrar los comandos de velocidad (v, ω) para estimar la posición
    del robot y calcular las velocidades angulares de cada rueda (wr, wl).

Modelo cinemático diferencial (integración de Euler):
    x(t+dt)  = x(t)  + v·cos(θ)·dt
    y(t+dt)  = y(t)  + v·sin(θ)·dt
    θ(t+dt)  = θ(t)  + ω·dt

    wr = (v + ω·L/2) / r      [rad/s — rueda derecha]
    wl = (v − ω·L/2) / r      [rad/s — rueda izquierda]

    donde:  r = radio de la rueda  [m]
            L = distancia entre ruedas (wheelbase)  [m]

Suscripciones:
    cmd_vel  (geometry_msgs/Twist)   — velocidad lineal y angular de entrada

Publicaciones:
    wr        (std_msgs/Float32)       — velocidad angular rueda derecha
    wl        (std_msgs/Float32)       — velocidad angular rueda izquierda
    pose_sim  (geometry_msgs/PoseStamped) — pose de verdad fundamental (ground truth)

Parámetros ROS 2:
    wheel_radius  [m]   (default 0.05)
    wheelbase     [m]   (default 0.19)
    sample_time   [s]   (default 0.02  → 50 Hz)
    x_init        [m]   (default 0.0)
    y_init        [m]   (default 0.0)
    theta_init    [rad] (default 0.0)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32


class KinematicSim(Node):
    """Simulador cinemático del Puzzlebot (modelo diferencial)."""

    def __init__(self):
        super().__init__('real_sim_robot')

        # ── Parámetros del robot ──────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)   # radio de la rueda [m]
        self.declare_parameter('wheelbase',    0.19)   # distancia entre ruedas [m]
        self.declare_parameter('sample_time',  0.02)   # periodo de integración [s]
        self.declare_parameter('x_init',       0.0)    # posición X inicial [m]
        self.declare_parameter('y_init',       0.0)    # posición Y inicial [m]
        self.declare_parameter('theta_init',   0.0)    # orientación inicial [rad]

        self.r  = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l  = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.dt = self.get_parameter('sample_time').get_parameter_value().double_value

        # ── Estado del robot (pose actual) ────────────────────────────────────
        self.x  = self.get_parameter('x_init').get_parameter_value().double_value
        self.y  = self.get_parameter('y_init').get_parameter_value().double_value
        self.th = self.get_parameter('theta_init').get_parameter_value().double_value

        # ── Entradas del controlador ──────────────────────────────────────────
        self.v = 0.0   # velocidad lineal  [m/s]
        self.w = 0.0   # velocidad angular [rad/s]

        # ── Comunicación ROS 2 ────────────────────────────────────────────────
        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_callback, 10)

        self.pub_wr   = self.create_publisher(Float32,     'wr',       10)
        self.pub_wl   = self.create_publisher(Float32,     'wl',       10)
        self.pub_pose = self.create_publisher(PoseStamped, 'pose_sim', 10)

        # ── Timer de integración (período fijo = sample_time) ─────────────────
        self.timer = self.create_timer(self.dt, self._timer_callback)

        self.get_logger().info(
            f'Simulador Cinemático iniciado | r={self.r} m  L={self.l} m  '
            f'dt={self.dt} s  pose_init=({self.x:.2f}, {self.y:.2f}, {self.th:.2f})'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_callback(self, msg: Twist):
        """Almacena el último comando de velocidad recibido."""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _timer_callback(self):
        """Integra la cinemática y publica estado en cada período."""

        # 1. Integración de Euler — actualiza la pose
        self.x  += self.v * math.cos(self.th) * self.dt
        self.y  += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt

        # 2. Cinemática inversa — velocidades angulares de rueda [rad/s]
        #    vR = v + (ω·L/2)  →  wr = vR / r
        #    vL = v − (ω·L/2)  →  wl = vL / r
        wr = (self.v + self.w * self.l / 2.0) / self.r
        wl = (self.v - self.w * self.l / 2.0) / self.r

        # 3. Publicar velocidades de rueda
        msg_wr      = Float32()
        msg_wr.data = float(wr)
        self.pub_wr.publish(msg_wr)

        msg_wl      = Float32()
        msg_wl.data = float(wl)
        self.pub_wl.publish(msg_wl)

        # 4. Publicar pose (ground truth) como PoseStamped
        #    El cuaternión se calcula desde el yaw: q = (0, 0, sin(θ/2), cos(θ/2))
        pose                        = PoseStamped()
        pose.header.stamp           = self.get_clock().now().to_msg()
        pose.header.frame_id        = 'odom'
        pose.pose.position.x        = self.x
        pose.pose.position.y        = self.y
        pose.pose.orientation.z     = math.sin(self.th / 2.0)
        pose.pose.orientation.w     = math.cos(self.th / 2.0)
        self.pub_pose.publish(pose)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSim()
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
