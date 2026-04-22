import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math
import math

class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')
        
        # --- Parameters ---
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheelbase', 0.19)
        self.declare_parameter('sample_time', 0.02)
        
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.dt = self.get_parameter('sample_time').get_parameter_value().double_value
        
        # --- State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # --- Inputs ---
        self.wr = 0.0
        self.wl = 0.0
        
        # --- Communication ---
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info("Localisation Node (Part 2) started")

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def timer_callback(self):
        # 1. Calculate v and w from wheel speeds
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.l
        
        # 2. Numerical Integration (Euler)
        self.x += v * math.cos(self.th) * self.dt
        self.y += v * math.sin(self.th) * self.dt
        self.th += w * self.dt
        
        # 3. Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
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
