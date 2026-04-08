import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math


class PuzzlebotKinematicSim(Node):
    """
    Nodo encargado de simular la cinematica del Puzzlebot.
    Calcula la posicion del robot en el plano y publica las transformaciones (TF)
    necesarias para que RViz lo dibuje moviendose.
    """

    def __init__(self):
        super().__init__("puzzlebot_kinematic_sim")

        # --- 1. CONFIGURACION DE MOVIMIENTO ---
        # Definimos velocidades constantes para crear una trayectoria circular
        self.v = 0.15  # Velocidad lineal (m/s)
        self.w = 0.35  # Velocidad angular (rad/s)

        # --- 2. ESTADO INICIAL DEL ROBOT ---
        # El robot empieza en el origen del mundo (0, 0, 0)
        self.x = 0.0  # Posicion en X
        self.y = 0.0  # Posicion en Y
        self.th = 0.0  # Orientacion (Angulo Theta en radianes)
        self.wheel_angle = 0.0  # Rotacion visual de las llantas

        # --- 3. COMUNICACION ROS2 ---
        # Publicador de TFs dinamicas (mueve el robot por el mapa)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicador de estados de articulacion (hace que las llantas giren)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # Configuramos el Timer: Se ejecuta cada 0.02s (50Hz) para un movimiento fluido
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("Simulador Cinematico de Puzzlebot Iniciado")

    def timer_callback(self):
        """
        Funcion principal que se repite constantemente para actualizar la fisica.
        """
        # --- A. CALCULO CINEMATICO (Integracion) ---
        # Basado en el modelo de robot diferencial:
        # 1. Actualizamos el angulo hacia donde mira el robot
        self.th += self.w * self.dt

        # 2. Descomponemos la velocidad lineal en X e Y usando trigonometria
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt

        # 3. Rotacion visual para las llantas (para que se vean girando en RViz)
        self.wheel_angle += (self.v / 0.05) * self.dt  # v / radio_llanta

        # Imprimimos la posicion actual en la terminal
        self.get_logger().info(
            f"Posicion: [{self.x:.2f}, {self.y:.2f}] | Theta: {self.th:.2f} rad"
        )

        # --- B. PUBLICAR TRANSFORMADA DINÁMICA (odom -> base_footprint) ---
        # Esta parte le dice a ROS: "El robot completo se movió a esta posición".
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"  # Marco de referencia fijo (El mundo)
        t.child_frame_id = "base_footprint"  # El origen del robot

        # Coordenadas calculadas arriba
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convertimos nuestro ángulo Theta a un Cuaternión (orientación en 3D)
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)

        # Enviamos la transformada al árbol de TFs
        self.tf_broadcaster.sendTransform(t)

        # --- C. PUBLICAR JOINT STATES (Giro de llantas) ---
        # Esto le dice a RViz cuánto deben girar los joints en el URDF
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["wheel_l_joint", "wheel_r_joint"]
        js.position = [self.wheel_angle, self.wheel_angle]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotKinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpieza al cerrar
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()