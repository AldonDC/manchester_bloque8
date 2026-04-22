import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math

class JointStatePublisher(Node):
    """
    Nodo "Joint State Publisher (user defined)" segun el diagrama del reto.
    Se suscribe a /odom para obtener la velocidad y calcular el giro de las ruedas.
    """
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # --- Parámetros (igual que en los otros nodos) ---
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheelbase', 0.19)
        self.declare_parameter('sample_time', 0.02)
        
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.dt = self.get_parameter('sample_time').get_parameter_value().double_value
        
        # --- Estado ---
        self.right_wheel_angle = 0.0
        self.left_wheel_angle = 0.0
        
        # --- Comunicación ---
        # Segun el diagrama, se suscribe a /odom
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.get_logger().info("Joint State Publisher (Part 2) iniciado (suscripto a /odom)")

    def odom_callback(self, msg):
        # 1. Obtener velocidades lineal y angular de la odometría
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        # 2. Reconvertir a velocidades de ruedas (rad/s)
        # vR = v + w*L/2, vL = v - w*L/2
        # wr = vR/r, wl = vL/r
        wr = (v + (w * self.l / 2.0)) / self.r
        wl = (v - (w * self.l / 2.0)) / self.r
        
        # 3. Integrar para obtener ángulo
        self.right_wheel_angle += wr * self.dt
        self.left_wheel_angle += wl * self.dt
        
        # 4. Publicar JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_r_joint', 'wheel_l_joint']
        js.position = [float(self.right_wheel_angle), float(self.left_wheel_angle)]
        
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
