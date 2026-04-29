"""
Nodo: localisation  (Parte 2 — Odometría por Dead Reckoning)
=============================================================
Estima la pose del robot a partir de las velocidades angulares de las ruedas.

Responsabilidad:
    Implementar el modelo cinemático inverso del Puzzlebot para reconstruir
    la velocidad lineal (v) y angular (ω) a partir de (wr, wl), e integrar
    la pose del robot en el tiempo. Publica el resultado como odometría estándar
    de ROS 2 (nav_msgs/Odometry).

Modelo cinemático inverso:
    v = r · (wr + wl) / 2          [m/s — velocidad lineal del centro]
    ω = r · (wr − wl) / L          [rad/s — velocidad angular]

Integración de Euler:
    x(t+dt)  = x(t)  + v·cos(θ)·dt
    y(t+dt)  = y(t)  + v·sin(θ)·dt
    θ(t+dt)  = θ(t)  + ω·dt

    donde:  r = radio de la rueda  [m]
            L = distancia entre ruedas (wheelbase)  [m]

Suscripciones:
    wr   (std_msgs/Float32)   — velocidad angular rueda derecha  [rad/s]
    wl   (std_msgs/Float32)   — velocidad angular rueda izquierda [rad/s]

Publicaciones:
    odom (nav_msgs/Odometry)  — pose y velocidad estimadas del robot

Parámetros ROS 2:
    wheel_radius  [m]   (default 0.05)
    wheelbase     [m]   (default 0.19)
    sample_time   [s]   (default 0.02  → 50 Hz)
    x_init        [m]   (default 0.0)
    y_init        [m]   (default 0.0)
    theta_init    [rad] (default 0.0)
    tf_prefix     [str] (default '')  — prefijo para frame IDs en multi-robot

Nota sobre tf_prefix:
    Los frame IDs de ROS 2 son strings globales; no se aíslan por namespace.
    En configuración multi-robot se deben prefixar manualmente para evitar
    colisiones en el árbol de TF. Ej: 'robot1/odom' → 'robot1/base_footprint'.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class Localisation(Node):
    """Estimador de pose por odometría de ruedas (dead reckoning)."""

    def __init__(self):
        super().__init__('localisation')

        # ── Parámetros del robot ──────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheelbase',    0.19)
        self.declare_parameter('sample_time',  0.02)
        self.declare_parameter('x_init',       0.0)
        self.declare_parameter('y_init',       0.0)
        self.declare_parameter('theta_init',   0.0)
        # tf_prefix: distingue los frames de cada robot en escenario multi-robot
        self.declare_parameter('tf_prefix',    '')

        self.r  = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l  = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.dt = self.get_parameter('sample_time').get_parameter_value().double_value

        # ── Frame IDs para la odometría ───────────────────────────────────────
        prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        # Si hay prefijo, los frames son "robotN/odom" y "robotN/base_footprint"
        self.odom_frame = f'{prefix}/odom'        if prefix else 'odom'
        self.base_frame = f'{prefix}/base_footprint' if prefix else 'base_footprint'

        # ── Estado del integrador (inicializado igual que kinematic_sim) ──────
        # Crítico: ambos nodos deben partir de la misma pose para evitar
        # discrepancias iniciales en el lazo de control.
        self.x  = self.get_parameter('x_init').get_parameter_value().double_value
        self.y  = self.get_parameter('y_init').get_parameter_value().double_value
        self.th = self.get_parameter('theta_init').get_parameter_value().double_value

        # ── Últimas lecturas de velocidad de rueda ────────────────────────────
        self.wr = 0.0   # velocidad rueda derecha  [rad/s]
        self.wl = 0.0   # velocidad rueda izquierda [rad/s]

        # ── Comunicación ROS 2 ────────────────────────────────────────────────
        self.sub_wr  = self.create_subscription(Float32, 'wr', self._wr_callback, 10)
        self.sub_wl  = self.create_subscription(Float32, 'wl', self._wl_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        # ── Timer de integración ──────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._timer_callback)

        self.get_logger().info(
            f'Localización iniciada | frames: {self.odom_frame} → {self.base_frame} | '
            f'pose_init=({self.x:.2f}, {self.y:.2f}, {self.th:.2f})'
        )

    # ── Callbacks de suscripción ──────────────────────────────────────────────

    def _wr_callback(self, msg: Float32):
        """Actualiza la velocidad angular de la rueda derecha."""
        self.wr = msg.data

    def _wl_callback(self, msg: Float32):
        """Actualiza la velocidad angular de la rueda izquierda."""
        self.wl = msg.data

    # ── Lazo de estimación ────────────────────────────────────────────────────

    def _timer_callback(self):
        """Calcula la pose por odometría y publica el mensaje Odometry."""

        # 1. Modelo cinemático inverso → v y ω desde velocidades de rueda
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.l

        # 2. Integración de Euler → actualiza la pose estimada
        self.x  += v * math.cos(self.th) * self.dt
        self.y  += v * math.sin(self.th) * self.dt
        self.th += w * self.dt

        # 3. Construir y publicar mensaje Odometry estándar de ROS 2
        odom                            = Odometry()
        odom.header.stamp               = self.get_clock().now().to_msg()
        odom.header.frame_id            = self.odom_frame    # frame padre
        odom.child_frame_id             = self.base_frame    # frame del robot

        # Pose estimada (cuaternión desde yaw: q = (0, 0, sin(θ/2), cos(θ/2)))
        odom.pose.pose.position.x       = self.x
        odom.pose.pose.position.y       = self.y
        odom.pose.pose.orientation.z    = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w    = math.cos(self.th / 2.0)

        # Velocidades instantáneas (usadas por joint_state_publisher para animar ruedas)
        odom.twist.twist.linear.x       = v
        odom.twist.twist.angular.z      = w

        self.pub_odom.publish(odom)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
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
