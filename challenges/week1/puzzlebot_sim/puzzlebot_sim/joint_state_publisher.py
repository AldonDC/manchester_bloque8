import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_puzzlebot')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0
        self.get_logger().info('🦾 Puzzlebot Joint States Publisher is active!')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_l_joint', 'wheel_r_joint']
        self.angle += 0.05 # Increment angle for rotation simulation
        if self.angle > 2 * math.pi:
            self.angle = 0.0
        msg.position = [self.angle, self.angle]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()