import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CoordinateTransform(Node):
    """
    Nodo "Coordinate Transform" según el diagrama.
    Transforma la odometría pura en una señal TF (odom -> base_footprint).
    """
    def __init__(self):
        super().__init__('coordinate_transform')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('Coordinate Transform iniciado (odom -> tf)')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # Asumiendo que el mapa está en el origen y odom es nuestro punto de referencia
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint' # Este es el que pide RViz
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransform()
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
