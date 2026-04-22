import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import math

class KinematicSim(Node):
    def __init__(self):
        super().__init__('real_sim_robot')
        
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
        
        # --- Input Variables ---
        self.v = 0.0
        self.w = 0.0
        
        # --- Communication ---
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info("Kinematic Simulator (Part 1) started")

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def timer_callback(self):
        # 1. Numerical Integration (Euler)
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt
        
        # 2. Calculate Wheel Speeds (rad/s)
        # vR = v + (w*l/2), vL = v - (w*l/2)
        # wr = vR/r, wl = vL/r
        wr = (self.v + (self.w * self.l / 2.0)) / self.r
        wl = (self.v - (self.w * self.l / 2.0)) / self.r
        
        # 3. Publish Wheel Speeds
        msg_wr = Float32()
        msg_wr.data = float(wr)
        self.wr_pub.publish(msg_wr)
        
        msg_wl = Float32()
        msg_wl.data = float(wl)
        self.wl_pub.publish(msg_wl)
        
        # 4. Publish PoseStamped (Simulated Ground Truth)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom' # Inertial frame
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        # Quaternion from Yaw
        pose_msg.pose.orientation.z = math.sin(self.th / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.th / 2.0)
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Solo intentamos apagar si rclpy sigue activo
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
