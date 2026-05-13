"""
Nodo: joint_state_publisher  —  Animación de ruedas
====================================================
Mini Challenge 4 · Manchester Robotics

¿POR QUÉ EXISTE?
    El URDF del Puzzlebot tiene joints rotacionales para wheel_l y wheel_r.
    Para que esos joints giren visualmente en RViz, ROS necesita recibir
    el ángulo acumulado de cada rueda en /joint_states. Este nodo se
    encarga de calcular esos ángulos a partir de la odometría.

LO QUE HACE EN CADA Δt:
    (a) Lee (v, ω) del /odom
    (b) Reconstruye la velocidad angular de cada rueda:
            ω_r = (v + ω·L/2) / r
            ω_l = (v − ω·L/2) / r
    (c) Integra el ángulo:
            θ_rueda(t+Δt) = θ_rueda(t) + ω_rueda · Δt
    (d) Publica /joint_states; robot_state_publisher actualiza el URDF.

CONTRATO ROS:
    Subscribe:  odom         (nav_msgs/Odometry)
    Publica:    joint_states (sensor_msgs/JointState)
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class JointStatePublisher(Node):
    """Publicador de estado de joints para animar ruedas en RViz."""

    def __init__(self):
        super().__init__('joint_state_publisher')

        # ── Parámetros del robot ──────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheelbase',    0.19)
        self.declare_parameter('sample_time',  0.02)
        # tf_prefix debe coincidir con el prefijo usado al modificar el URDF
        self.declare_parameter('tf_prefix',    '')

        self.r  = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l  = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.dt = self.get_parameter('sample_time').get_parameter_value().double_value
        prefix  = self.get_parameter('tf_prefix').get_parameter_value().string_value

        # ── Nombres de joint (deben coincidir con el URDF prefixado) ──────────
        if prefix:
            self.joint_r = f'{prefix}/wheel_r_joint'
            self.joint_l = f'{prefix}/wheel_l_joint'
        else:
            self.joint_r = 'wheel_r_joint'
            self.joint_l = 'wheel_l_joint'

        # ── Ángulos acumulados de cada rueda ──────────────────────────────────
        self.angle_r = 0.0   # ángulo rueda derecha  [rad]
        self.angle_l = 0.0   # ángulo rueda izquierda [rad]

        # ── Comunicación ROS 2 ────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)

        self.get_logger().info(
            f'Joint State Publisher iniciado | '
            f'joints: [{self.joint_r}, {self.joint_l}]'
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        """Integra el ángulo de cada rueda y publica el JointState."""

        # 1. Extraer velocidades del robot desde el mensaje de odometría
        v = msg.twist.twist.linear.x    # velocidad lineal  [m/s]
        w = msg.twist.twist.angular.z   # velocidad angular [rad/s]

        # 2. Convertir a velocidades angulares de rueda
        #    vR = v + (ω·L/2)  →  wr = vR / r
        #    vL = v − (ω·L/2)  →  wl = vL / r
        wr = (v + w * self.l / 2.0) / self.r
        wl = (v - w * self.l / 2.0) / self.r

        # 3. Integrar ángulo acumulado de cada rueda
        self.angle_r += wr * self.dt
        self.angle_l += wl * self.dt

        # 4. Publicar JointState para robot_state_publisher
        js           = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name      = [self.joint_r, self.joint_l]
        js.position  = [float(self.angle_r), float(self.angle_l)]

        self.pub_js.publish(js)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
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
