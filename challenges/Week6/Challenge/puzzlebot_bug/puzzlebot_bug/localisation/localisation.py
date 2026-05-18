"""
Nodo: localisation  —  Núcleo del Mini Challenge 4
==================================================
Manchester Robotics × NVIDIA · Week 5

¿QUÉ HACE ESTE NODO?
--------------------
Estima la pose del Puzzlebot por DEAD RECKONING (integrando velocidades de
rueda) y, al mismo tiempo, calcula cuánta incertidumbre se ha acumulado en
esa estimación. La incertidumbre se publica como matriz de covarianza Σ_k
dentro del mensaje /odom para que RViz la dibuje automáticamente como una
elipse alrededor del robot.

LO QUE PIDE EL RETO (PDF · Task 1):
    "Plot the covariance ellipsoid for the robot's pose using the
     uncertainty propagation model... complete the 3x3 pose covariance
     matrix of the 'odometry' message in the localisation node."

MODELO PROBABILÍSTICO (sección "Tips" del PDF):
    s_k ~ N(μ_k, Σ_k)
        μ_k  =  pose más probable (dead reckoning)
        Σ_k  =  H_k · Σ_{k-1} · H_kᵀ  +  Q_k     ← se propaga en cada ciclo

    Q_k    = ∇ω_k · Σ_Δ,k · ∇ω_kᵀ
    ∇ω_k   = Jacobiano 3x2 respecto a (ω_r, ω_l)
    Σ_Δ,k  = diag(k_r·|ω_r|,  k_l·|ω_l|)        ← ruido de cada rueda

PARÁMETROS CALIBRABLES (Task 2):
    k_r, k_l  →  ganancias de ruido por rueda. Se ajustan con experimentos
                 reales hasta que la elipse cubra la dispersión observada.

ROS:
    Subscribe:  wr   (std_msgs/Float32)  — velocidad rueda derecha  [rad/s]
                wl   (std_msgs/Float32)  — velocidad rueda izquierda [rad/s]
    Publica:    odom (nav_msgs/Odometry) — pose + velocidades + covarianza

DATAFLOW (50 Hz):
    wr,wl  ──►  (v,ω)  ──►  integra μ_k  ──►  propaga Σ_k  ──►  publica /odom
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class Localisation(Node):
    """Estimador de pose + covarianza por dead reckoning (Mini Challenge 4)."""

    def __init__(self):
        super().__init__('localisation')

        # ──────────────────────────────────────────────────────────────────────
        # PARÁMETROS DECLARADOS
        # ──────────────────────────────────────────────────────────────────────
        # Geometría física del Puzzlebot. Cambiar aquí si el robot real difiere.
        self.declare_parameter('wheel_radius', 0.05)   # r [m]
        self.declare_parameter('wheelbase',    0.19)   # L [m] (distancia entre ruedas)
        self.declare_parameter('sample_time',  0.02)   # Δt [s] → 50 Hz

        # Pose inicial (debe coincidir con el simulador para que no haya offset).
        self.declare_parameter('x_init',     0.0)
        self.declare_parameter('y_init',     0.0)
        self.declare_parameter('theta_init', 0.0)

        # Ganancias de ruido del PDF (Task 2 → calibrar experimentalmente).
        # Valores típicos: 0.02 (poco ruido) … 0.20 (mucho ruido).
        self.declare_parameter('k_r', 0.05)
        self.declare_parameter('k_l', 0.05)

        # Prefijo para escenarios multi-robot (vacío = un solo robot).
        self.declare_parameter('tf_prefix', '')

        # Caché de valores numéricos para no consultar parámetros en cada ciclo.
        self.r   = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l   = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.dt  = self.get_parameter('sample_time').get_parameter_value().double_value
        self.k_r = self.get_parameter('k_r').get_parameter_value().double_value
        self.k_l = self.get_parameter('k_l').get_parameter_value().double_value

        # ──────────────────────────────────────────────────────────────────────
        # FRAMES TF
        # ──────────────────────────────────────────────────────────────────────
        # header.frame_id (padre) = "odom"   ← marco de referencia fijo
        # child_frame_id  (hijo)  = "base_footprint"  ← cuerpo del robot
        # RViz necesita este par para dibujar la pose y la elipse correctamente.
        prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.odom_frame = f'{prefix}/odom'           if prefix else 'odom'
        self.base_frame = f'{prefix}/base_footprint' if prefix else 'base_footprint'

        # ──────────────────────────────────────────────────────────────────────
        # ESTADO ESTIMADO
        # ──────────────────────────────────────────────────────────────────────
        # μ_k = (x, y, θ) — pose más probable
        self.x  = self.get_parameter('x_init').get_parameter_value().double_value
        self.y  = self.get_parameter('y_init').get_parameter_value().double_value
        self.th = self.get_parameter('theta_init').get_parameter_value().double_value

        # Σ_k inicia en 0: al arrancar conocemos la pose con certeza absoluta.
        # En dead reckoning Σ SÓLO crece (porque no hay observaciones que la
        # corrijan, como sí pasaría en un Kalman con LiDAR/cámara).
        self.Sigma = np.zeros((3, 3), dtype=float)

        # Últimas velocidades de rueda recibidas (entradas del modelo).
        self.wr = 0.0
        self.wl = 0.0

        # ──────────────────────────────────────────────────────────────────────
        # CONEXIONES ROS 2
        # ──────────────────────────────────────────────────────────────────────
        # Entrada: velocidades de rueda del simulador (Real/Sim Robot).
        self.sub_wr = self.create_subscription(Float32, 'wr', self._wr_callback, 10)
        self.sub_wl = self.create_subscription(Float32, 'wl', self._wl_callback, 10)

        # Salida: mensaje Odometry estándar con pose + covarianza.
        # Lo consumen: RViz (elipse), coordinate_transform (TF), joint_state_pub.
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        # Lazo a frecuencia fija (timer driven, no event driven).
        self.timer = self.create_timer(self.dt, self._timer_callback)

        self.get_logger().info(
            f'Localización + covarianza iniciada | '
            f'frames: {self.odom_frame} → {self.base_frame} | '
            f'k_r={self.k_r:.4f}  k_l={self.k_l:.4f} | '
            f'dt={self.dt}s'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACKS DE SUSCRIPCIÓN  (sólo guardan el dato, no hacen cálculos)
    # ─────────────────────────────────────────────────────────────────────────

    def _wr_callback(self, msg: Float32):
        """Almacena la última velocidad angular de la rueda derecha [rad/s]."""
        self.wr = msg.data

    def _wl_callback(self, msg: Float32):
        """Almacena la última velocidad angular de la rueda izquierda [rad/s]."""
        self.wl = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    # LAZO PRINCIPAL — ejecutado cada Δt segundos (50 Hz por defecto)
    # ─────────────────────────────────────────────────────────────────────────

    def _timer_callback(self):
        """
        Pipeline de cada ciclo:
            (1) reconstruir (v, ω) desde las velocidades de rueda
            (2) integrar la pose por Euler             →  actualiza μ_k
            (3) propagar la covarianza                 →  actualiza Σ_k
            (4) construir y publicar el mensaje Odometry
        """

        # (1) MODELO CINEMÁTICO INVERSO (rueda → robot)
        #     v = r · (ω_r + ω_l) / 2          velocidad lineal del centro [m/s]
        #     ω = r · (ω_r − ω_l) / L          velocidad angular [rad/s]
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.l

        # IMPORTANTE: el Jacobiano H y ∇ω se evalúan en θ_{k-1}, así que
        # guardamos el yaw ANTES de integrarlo.
        th_prev = self.th

        # (2) INTEGRACIÓN EULER 
        #     x(t+Δt) = x(t) + v·cos(θ)·Δt
        #     y(t+Δt) = y(t) + v·sin(θ)·Δt
        #     θ(t+Δt) = θ(t) + ω·Δt
        self.x  += v * math.cos(th_prev) * self.dt
        self.y  += v * math.sin(th_prev) * self.dt
        self.th += w * self.dt

        # (3) PROPAGACIÓN DE COVARIANZA — núcleo matemático del reto.
        self.Sigma = self._propagate_covariance(v, th_prev)

        # (4) PUBLICAR /odom CON μ_k Y Σ_k DENTRO.
        self._publish_odometry(v, w)

    # ─────────────────────────────────────────────────────────────────────────
    # PROPAGACIÓN DE INCERTIDUMBRE  ⭐  núcleo del Mini Challenge 
    # ─────────────────────────────────────────────────────────────────────────

    def _propagate_covariance(self, v: float, th_prev: float) -> np.ndarray:
        """
        Implementa la ecuación textual del PDF:

                Σ_k = H_k · Σ_{k-1} · H_kᵀ  +  Q_k

        Donde:
            H_k    = Jacobiano del modelo cinemático       (3x3)
            Q_k    = error no-determinístico de las ruedas (3x3)
                   = ∇ω_k · Σ_Δ,k · ∇ω_kᵀ
            ∇ω_k   = Jacobiano respecto a (ω_r, ω_l)        (3x2)
            Σ_Δ,k  = covarianza de las entradas             (2x2)

        El método NO usa scipy ni Kalman pre-hecho. Es álgebra a mano con
        NumPy puro, como lo pide el enunciado.
        """
        dt = self.dt
        r  = self.r
        L  = self.l
        c  = math.cos(th_prev)   # se evalúa en θ_{k-1}, NO en el θ recién actualizado
        s  = math.sin(th_prev)

        # ── H_k  (3x3) ───────────────────────────────────────────────────────
        # Derivada parcial de la pose nueva respecto a la pose vieja.
        # La columna de θ refleja cómo un error de orientación rota la posición.
        #
        #     ┌ 1   0   −Δt·v·sin(θ) ┐
        # H = │ 0   1    Δt·v·cos(θ) │
        #     └ 0   0          1     ┘
        H = np.array([
            [1.0, 0.0, -dt * v * s],
            [0.0, 1.0,  dt * v * c],
            [0.0, 0.0,         1.0],
        ], dtype=float)

        # ── ∇ω_k  (3x2) ──────────────────────────────────────────────────────
        # Derivada parcial de la pose respecto a las velocidades de cada rueda.
        # Factor (r·Δt/2) viene de que v = r·(ω_r + ω_l)/2  y  ω = r·(ω_r − ω_l)/L.
        #
        #            r·Δt    ┌  cos θ   cos θ ┐
        #  ∇ω_k  =  ─────  · │  sin θ   sin θ │
        #              2     └  2/L    −2/L   ┘
        grad_w = 0.5 * r * dt * np.array([
            [c,        c       ],
            [s,        s       ],
            [2.0 / L, -2.0 / L ],
        ], dtype=float)

        # ── Σ_Δ,k  (2x2) ─────────────────────────────────────────────────────
        # Covarianza del RUIDO en las entradas. Modelo del PDF:
        # cada rueda tiene ruido proporcional a la magnitud de su velocidad.
        # Si la rueda no se mueve (|ω|=0), no aporta ruido.
        Sigma_delta = np.array([
            [self.k_r * abs(self.wr), 0.0                     ],
            [0.0,                     self.k_l * abs(self.wl) ],
        ], dtype=float)

        # ── Q_k = ∇ω · Σ_Δ · ∇ωᵀ  (3x3) ──────────────────────────────────────
        # Lleva la incertidumbre del espacio de ruedas al espacio de la pose.
        Q = grad_w @ Sigma_delta @ grad_w.T

        # ── Σ_k = H · Σ_{k-1} · Hᵀ + Q  (3x3) ────────────────────────────────
        # Primer término: propaga la incertidumbre previa a través del modelo.
        # Segundo término: agrega la incertidumbre nueva inyectada este Δt.
        return H @ self.Sigma @ H.T + Q

    # ─────────────────────────────────────────────────────────────────────────
    # PUBLICACIÓN DEL MENSAJE Odometry
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_odometry(self, v: float, w: float):
        """
        Construye nav_msgs/Odometry con:
            - header.frame_id    = 'odom'           (frame padre)
            - child_frame_id     = 'base_footprint' (frame del robot)
            - pose.pose          = (x, y, yaw)      (μ_k)
            - pose.covariance    = Σ_k mapeada al arreglo de 36 floats
            - twist.twist        = (v, ω)
        """
        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        # ── Pose media μ_k ──
        # Como el robot es 2D, sólo se llena (x, y) y un cuaternión de yaw.
        # Yaw → cuaternión:  q = (0, 0, sin(θ/2), cos(θ/2))
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        # ── Mapeo Σ_k (3x3) → pose.covariance (36 floats)  ⭐ ──
        # nav_msgs/Odometry guarda la covarianza como una 6x6 aplanada
        # sobre (x, y, z, roll, pitch, yaw). El Puzzlebot es 2D, así que
        # SÓLO llenamos los 9 cruces (x, y, θ) y dejamos el resto en cero.
        #
        # Layout del arreglo plano (índices entre paréntesis):
        #                x      y      z     φ     ψ      θ
        #          x  [  0      1      2     3     4      5  ]
        #          y  [  6      7      8     9    10     11  ]
        #          z  [ 12     13     14    15    16     17  ]
        #          φ  [ 18     19     20    21    22     23  ]
        #          ψ  [ 24     25     26    27    28     29  ]
        #          θ  [ 30     31     32    33    34     35  ]
        cov = [0.0] * 36
        cov[0]  = float(self.Sigma[0, 0])   # σ_xx
        cov[1]  = float(self.Sigma[0, 1])   # σ_xy
        cov[5]  = float(self.Sigma[0, 2])   # σ_xθ
        cov[6]  = float(self.Sigma[1, 0])   # σ_yx
        cov[7]  = float(self.Sigma[1, 1])   # σ_yy
        cov[11] = float(self.Sigma[1, 2])   # σ_yθ
        cov[30] = float(self.Sigma[2, 0])   # σ_θx
        cov[31] = float(self.Sigma[2, 1])   # σ_θy
        cov[35] = float(self.Sigma[2, 2])   # σ_θθ
        odom.pose.covariance = cov

        # ── Twist (velocidades) ──
        # El PDF permite dejar la covarianza de velocidad en CERO.
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        self.pub_odom.publish(odom)


# ─────────────────────────────────────────────────────────────────────────────
# PUNTO DE ENTRADA  (ros2 run / launch lo invoca aquí)
# ─────────────────────────────────────────────────────────────────────────────

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
