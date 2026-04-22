import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

class TrajectoryGenerator(Node):
    """
    Nodo "Set Point Generator" (Extra de la Parte 3).
    Manda una secuencia de puntos al controlador y pone esferas en RViz.
    """
    def __init__(self):
        super().__init__('trajectory_set_point_generator')
        
        # --- Parámetro para elegir la figura ---
        self.declare_parameter('shape', 'square')
        shape = self.get_parameter('shape').value
        
        # --- Trayectorias definibles ---
        if shape == 'triangle':
            # Triángulo equilátero
            self.points = [(1.5, 0.0), (0.75, 1.3), (0.0, 0.0)]
        elif shape == 'hexagon':
            # Hexágono
            self.points = [(1.0, 0.0), (1.5, 0.86), (1.0, 1.73), (0.0, 1.73), (-0.5, 0.86), (0.0, 0.0)]
        else:
            # Cuadrado por defecto
            self.points = [(1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0)]
            
        self.current_point_idx = 0
        
        # --- Comunicación ---
        self.pub_goal = self.create_publisher(Point, 'set_point', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.sub_reached = self.create_subscription(Bool, 'next_point', self.reached_callback, 10)
        
        self.waiting_for_goal = False
        self.timer_init = self.create_timer(2.0, self.initial_publish)
        
        # Timer para publicar los marcadores constantemente
        self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info(f"Generador de Trayectorias iniciado. Forma: {shape.upper()}")

    def initial_publish(self):
        self.send_next_point()
        self.timer_init.cancel()

    def reached_callback(self, msg):
        # Solo avanzamos si el mensaje es True y no estamos ya esperando
        if msg.data and not self.waiting_for_goal:
            self.get_logger().info(f"¡Esquina {self.current_point_idx} alcanzada!")
            self.current_point_idx = (self.current_point_idx + 1) % len(self.points)
            self.waiting_for_goal = True
            # Usamos un timer de un solo disparo
            self.one_shot_timer = self.create_timer(2.0, self.timer_finished)

    def timer_finished(self):
        self.one_shot_timer.cancel()
        self.send_next_point()

    def send_next_point(self):
        target = self.points[self.current_point_idx]
        msg = Point(x=target[0], y=target[1], z=0.0)
        self.pub_goal.publish(msg)
        self.waiting_for_goal = False
        self.get_logger().info(f"Siguiente objetivo: ({msg.x}, {msg.y})")

    def publish_markers(self):
        ma = MarkerArray()
        for i, p in enumerate(self.points):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.pose.position.z = 0.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            # El punto actual es verde, los otros amarillos
            if i == self.current_point_idx:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.6
            ma.markers.append(m)
        self.pub_markers.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
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
