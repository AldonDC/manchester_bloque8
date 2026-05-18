"""
wheel_state_bridge — Extrae velocidades de rueda desde /joint_states
======================================================================
Mini Challenge · Reactive Navigation · Week 6

¿POR QUÉ EXISTE?
    El simulador MCR2 publica /joint_states (sensor_msgs/JointState) con la
    velocidad de cada rueda. El nodo localisation del Reto 4 espera
    /wr y /wl como std_msgs/Float32 individuales. Este bridge convierte
    uno en el otro sin modificar al localisation.

CONTRATO ROS:
    Subscribe: joint_states  (sensor_msgs/JointState)
    Publica:   /wr           (std_msgs/Float32)  velocidad rueda derecha
               /wl           (std_msgs/Float32)  velocidad rueda izquierda

Parámetros:
    wheel_right_name   nombre del joint derecho (default 'wheel_right_joint')
    wheel_left_name    nombre del joint izquierdo (default 'wheel_left_joint')
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class WheelStateBridge(Node):
    """Convierte /joint_states del simulador a los topics /wr y /wl que
    espera el nodo localisation del Reto 4."""

    def __init__(self):
        super().__init__('wheel_state_bridge')

        self.declare_parameter('wheel_right_name', 'wheel_right_joint')
        self.declare_parameter('wheel_left_name',  'wheel_left_joint')

        self.right_name = self.get_parameter('wheel_right_name').get_parameter_value().string_value
        self.left_name  = self.get_parameter('wheel_left_name').get_parameter_value().string_value

        self.pub_wr = self.create_publisher(Float32, 'wr', 10)
        self.pub_wl = self.create_publisher(Float32, 'wl', 10)

        self.sub_js = self.create_subscription(
            JointState, 'joint_states', self._js_callback, 10)

        self.get_logger().info(
            f'Wheel state bridge | R="{self.right_name}"  L="{self.left_name}"')

    def _js_callback(self, msg: JointState):
        """Extrae la velocidad de cada rueda y la republica como Float32."""
        # Mapear nombres a sus velocidades (msg.velocity es paralelo a msg.name)
        if not msg.name or not msg.velocity:
            return
        for name, vel in zip(msg.name, msg.velocity):
            if name == self.right_name:
                self.pub_wr.publish(Float32(data=float(vel)))
            elif name == self.left_name:
                self.pub_wl.publish(Float32(data=float(vel)))


def main(args=None):
    rclpy.init(args=args)
    node = WheelStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
