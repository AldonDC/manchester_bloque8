import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import math

class PositionController(Node):
    """
    Nodo "control" (Parte 3). 
    Implementa un controlador PID para llevar al robot a una coordenada (x, y).
    """
    def __init__(self):
        super().__init__('controller')
        
        # --- Parámetros PID ---
        # Lineal (v)
        self.declare_parameter('k_v', 0.5)
        # Angular (w) - PID para orientación
        self.declare_parameter('kp_w', 1.0)
        self.declare_parameter('ki_w', 0.01)
        self.declare_parameter('kd_w', 0.1)
        
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('dist_tolerance', 0.05)
        
        # --- Estado del PID ---
        self.prev_angle_error = 0.0
        self.integral_angle_error = 0.0
        
        # --- Estado actual (del odom) ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_th = 0.0
        
        # --- Comunicación ---
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_target = self.create_subscription(Point, 'set_point', self.target_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_reached = self.create_publisher(Bool, 'next_point', 10)
        
        # --- VARIABLES DE ESTADO PARA TRANSICIONES ---
        self.goal_reached_sent = False
        
        # Timer para el lazo de control (50Hz)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Controlador PID (Parte 3) iniciado.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Yaw extraction
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_th = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        # Resetear términos integral y estado de llegada al recibir un punto NUEVO
        self.integral_angle_error = 0.0
        self.prev_angle_error = 0.0
        self.goal_reached_sent = False # Permitir avisar cuando llegue a este nuevo punto
        
        params = [
            rclpy.parameter.Parameter('target_x', rclpy.Parameter.Type.DOUBLE, msg.x),
            rclpy.parameter.Parameter('target_y', rclpy.Parameter.Type.DOUBLE, msg.y)
        ]
        self.set_parameters(params)
        self.get_logger().info(f"Nuevo objetivo recibido: x={msg.x}, y={msg.y}")

    def control_loop(self):
        tx = self.get_parameter('target_x').value
        ty = self.get_parameter('target_y').value
        kv = self.get_parameter('k_v').value
        kp_w = self.get_parameter('kp_w').value
        ki_w = self.get_parameter('ki_w').value
        kd_w = self.get_parameter('kd_w').value
        tol = self.get_parameter('dist_tolerance').value

        # 1. Errores
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.current_th
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) # Normalizar

        msg = Twist()
        reached_msg = Bool(data=False)

        if dist > tol:
            # PID Angular
            self.integral_angle_error += angle_error * self.dt
            derivative = (angle_error - self.prev_angle_error) / self.dt
            
            w = (kp_w * angle_error) + (ki_w * self.integral_angle_error) + (kd_w * derivative)
            
            # Mejora de alineación (primero gira, luego avanza)
            if abs(angle_error) > 0.2:
                v = 0.0 
            else:
                v = kv * dist
                
            msg.linear.x = max(min(v, 0.3), 0.0) 
            msg.angular.z = max(min(w, 1.5), -1.5) 
            
            self.prev_angle_error = angle_error
        else:
            # ¡Llegamos al punto!
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            # Solo mandamos el "True" una vez por cada punto
            if not self.goal_reached_sent:
                reached_msg.data = True
                self.goal_reached_sent = True # Bloqueamos futuras señales hasta el siguiente punto
            
            self.integral_angle_error = 0.0
            
        self.pub_cmd.publish(msg)
        self.pub_reached.publish(reached_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
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
