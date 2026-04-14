import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math


class PuzzlebotKinematicSim(Node):
    """
    Nodo encargado de simular la cinematica del Puzzlebot.
    Calcula la posicion del robot en el plano y publica las 6 transformaciones (TF)
    explícitamente para cumplir con el reto de la Semana 2.
    """

    def __init__(self):
        super().__init__("puzzlebot_kinematic_sim")

        # --- 1. CONFIGURACION DE MOVIMIENTO ---
        self.v = 0.15 
        self.w = 0.35 

        # --- 2. ESTADO INICIAL ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.wheel_angle = 0.0

        # --- 3. COMUNICACION ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("Simulador Cinematico Iniciado (Manual TF Mode)")

    def publish_tf(self, parent_id, child_id, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_id
        t.child_frame_id = child_id
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        # Actualización de cinemática
        self.th += self.w * self.dt
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.wheel_angle += (self.v / 0.05) * self.dt

        # 1. map -> odom
        self.publish_tf("map", "odom")

        # 2. odom -> base_footprint (Yaw rotation)
        self.publish_tf("odom", "base_footprint", x=self.x, y=self.y, 
                        qz=math.sin(self.th/2), qw=math.cos(self.th/2))

        # 3. base_footprint -> base_link
        self.publish_tf("base_footprint", "base_link", z=0.05)

        # 4. base_link -> wheel_l (Pitch rotation around local Y)
        self.publish_tf("base_link", "wheel_l", x=0.052, y=0.095, z=-0.0025,
                        qy=math.sin(self.wheel_angle/2), qw=math.cos(self.wheel_angle/2))

        # 5. base_link -> wheel_r (Yaw 180 + Pitch rotation)
        # Para que gire hacia adelante al estar invertida, invertimos el signo del componente X
        s = math.sin(self.wheel_angle / 2.0)
        c = math.cos(self.wheel_angle / 2.0)
        self.publish_tf("base_link", "wheel_r", x=0.052, y=-0.095, z=-0.0025,
                        qx=s, qy=0.0, qz=c, qw=0.0)

        # 6. base_link -> caster
        self.publish_tf("base_link", "caster", x=-0.095, y=0.0, z=-0.03)

        # JointStates (opcional)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["wheel_l_joint", "wheel_r_joint"]
        js.position = [self.wheel_angle, self.wheel_angle]
        self.joint_pub.publish(js)

        # Logging
        self.get_logger().info(f"Pose: x={self.x:.2f}, y={self.y:.2f}, th={self.th:.2f}")


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