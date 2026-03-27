import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class PuzzlebotKinematicSim(Node):
    def __init__(self):
        super().__init__("puzzlebot_kinematic_sim")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Robot State (X, Y, Theta)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.wheel_angle = 0.0

        # Parameters (Constant movement for demo)
        self.v = 0.15  # 0.15 m/s
        self.w = 0.35  # 0.35 rad/s
        self.r = 0.05  # Wheel radius

        self.get_logger().info("🚀 Puzzlebot PRO Sim: Moving and publishing TFs!")

    def timer_callback(self):
        dt = 0.02

        # 1. Update Kinematics (Circular motion)
        self.th += self.w * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.wheel_angle += (self.v / self.r) * dt

        # 2. Log to terminal (as requested)
        self.get_logger().info(
            f"📍 Pos: [{self.x:.2f}, {self.y:.2f}] | Θ: {self.th:.2f} rad"
        )

        # 3. Publish JointState (for URDF wheels standing up)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["wheel_l_joint", "wheel_r_joint"]
        js.position = [self.wheel_angle, self.wheel_angle]
        self.joint_pub.publish(js)

        # 4. Publish Global Transform (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # RPY to Quaternion (Z-axis rotation only)
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotKinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()