"""
Nodo: kinematic_sim  —  Real / Sim Robot
=========================================
Mini Challenge 4 · Manchester Robotics

ROL EN LA ARQUITECTURA (círculo azul del diagrama del PDF):
    Es la "planta física virtual". Recibe comandos de velocidad del
    Controller y genera las velocidades de rueda que ve la Localisation.
    Sirve como sustituto del Puzzlebot real para hacer pruebas sin hardware.

QUÉ HACE EN CADA CICLO (50 Hz):
    (a) Integra (v, ω) → actualiza la pose ground truth (x, y, θ)
    (b) Calcula las velocidades de rueda (ω_r, ω_l) por cinemática inversa
    (c) Publica /wr, /wl y la pose ground truth /pose_sim

ECUACIONES (modelo diferencial, integradas por Euler):
    Pose:
        x(t+Δt) = x(t) + v·cos(θ)·Δt
        y(t+Δt) = y(t) + v·sin(θ)·Δt
        θ(t+Δt) = θ(t) + ω·Δt

    Cinemática inversa (robot → rueda):
        ω_r = (v + ω·L/2) / r
        ω_l = (v − ω·L/2) / r

    donde  r = radio de rueda [m],  L = wheelbase [m]

ROS:

    Subscribe:  cmd_vel  (geometry_msgs/Twist)         — comando del Controller
    Publica:    wr       (std_msgs/Float32)            — entrada de Localisation
                wl       (std_msgs/Float32)            — entrada de Localisation
                pose_sim (geometry_msgs/PoseStamped)   — ground truth para comparación
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
        """Guarda el último comando (v, ω) que llega del Controller."""
        self.v = msg.linear.x     # velocidad lineal del centro del robot
        self.w = msg.angular.z    # velocidad angular (yaw rate)

    def _timer_callback(self):
        """Avanza la simulación un paso de Δt y publica las salidas."""

        # (1) INTEGRACIÓN DE EULER  →  nueva pose ground truth
        #     "Solving ODEs by hand" como pide el PDF, sin scipy.
        self.x  += self.v * math.cos(self.th) * self.dt
        self.y  += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt

        # (2) CINEMÁTICA INVERSA  →  velocidad angular de cada rueda
        #     Velocidad lineal de cada rueda (vR, vL):
        #         vR = v + ω·L/2     (la rueda externa va más rápido al girar)
        #         vL = v − ω·L/2
        #     Velocidad angular:  ω_rueda = v_rueda / r
        wr = (self.v + self.w * self.l / 2.0) / self.r
        wl = (self.v - self.w * self.l / 2.0) / self.r

        # (3) PUBLICAR /wr y /wl  →  son la entrada de Localisation
        msg_wr      = Float32()
        msg_wr.data = float(wr)
        self.pub_wr.publish(msg_wr)

        msg_wl      = Float32()
        msg_wl.data = float(wl)
        self.pub_wl.publish(msg_wl)

        # (4) PUBLICAR LA POSE GROUND TRUTH  →  sirve para comparar contra
        #     la pose estimada por Localisation y validar la elipse Σ_k.
        #     Yaw → cuaternión:  q = (0, 0, sin(θ/2), cos(θ/2))
        pose                    = PoseStamped()
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.header.frame_id    = 'odom'
        pose.pose.position.x    = self.x
        pose.pose.position.y    = self.y
        pose.pose.orientation.z = math.sin(self.th / 2.0)
        pose.pose.orientation.w = math.cos(self.th / 2.0)
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
