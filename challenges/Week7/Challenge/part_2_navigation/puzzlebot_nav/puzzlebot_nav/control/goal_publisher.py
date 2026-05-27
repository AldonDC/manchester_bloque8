"""
goal_publisher — Publicador de objetivo único
==============================================
Mini Challenge · Reactive Navigation (Week 6)

¿QUÉ HACE?
    Publica una sola coordenada (x_T, y_T) en /set_point al arrancar.
    Es el equivalente al trajectory_generator de retos previos, pero
    con un único waypoint (los Bug algorithms solo necesitan un goal).

    Además republica el objetivo como un Marker (esfera verde) para que
    se vea en RViz.

CONTRATO ROS:
    Publica: /set_point                  (geometry_msgs/Point)
             /visualization_marker_array (visualization_msgs/MarkerArray)

PARÁMETROS:
    target_x   coordenada X del objetivo  [m]   (default 3.0)
    target_y   coordenada Y del objetivo  [m]   (default 0.0)
    frame_id   frame del Marker (default 'odom')
    publish_period  cada cuántos segundos re-publicar (default 1.0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class GoalPublisher(Node):
    """Publica el objetivo (x_T, y_T) y un marcador para RViz."""

    def __init__(self):
        super().__init__('goal_publisher')

        self.declare_parameter('target_x',       3.0)
        self.declare_parameter('target_y',       0.0)
        self.declare_parameter('frame_id',       'odom')
        self.declare_parameter('publish_period', 1.0)

        self.tx = self.get_parameter('target_x').value
        self.ty = self.get_parameter('target_y').value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        period = self.get_parameter('publish_period').value

        # /goal: objetivo final que recibe el nodo Bug.
        # NO confundir con /set_point — ese lo usa el Bug para mandar
        # sub-waypoints intermedios al controller PID.
        self.pub_goal   = self.create_publisher(Point, 'goal', 10)
        self.pub_marker = self.create_publisher(
            MarkerArray, 'visualization_marker_array', 10)

        self.timer = self.create_timer(period, self._publish_all)

        self.get_logger().info(
            f'Goal Publisher | target=({self.tx:.2f}, {self.ty:.2f})  '
            f'frame={self.frame_id}'
        )

    def _publish_all(self):
        # /set_point — geometry_msgs/Point
        p = Point()
        p.x = float(self.tx)
        p.y = float(self.ty)
        p.z = 0.0
        self.pub_goal.publish(p)

        # MarkerArray — esfera verde para RViz
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.tx)
        m.pose.position.y = float(self.ty)
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        ma.markers.append(m)
        self.pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
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
