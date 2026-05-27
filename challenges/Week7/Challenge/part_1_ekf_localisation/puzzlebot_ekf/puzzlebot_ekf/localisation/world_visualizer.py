"""
world_visualizer — Publishes the hamster maze geometry to RViz as MarkerArray.
Mirrors every wall of ekf_arena.world so the RViz view matches Gazebo.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


_OUTER = [
    ("wall_south",  2.5, -1.0, 0.30,  7.1, 0.06, 0.6),
    ("wall_north",  2.5,  5.0, 0.30,  7.1, 0.06, 0.6),
    ("wall_west",  -1.0,  2.0, 0.30,  0.06, 6.1, 0.6),
    ("wall_east",   6.0,  2.0, 0.30,  0.06, 6.1, 0.6),
]

_INNER = [
    ("iv1_west",   0.5,  3.5,  0.30, 0.06, 1.6, 0.6),
    ("iv1_east",   1.8,  3.5,  0.30, 0.06, 1.6, 0.6),
    ("iv1_top",    1.15, 4.25, 0.30, 1.3,  0.06, 0.6),
    ("ih1",        0.95, 3.6,  0.30, 0.85, 0.06, 0.6),
    ("iv4_west",   3.2,  3.5,  0.30, 0.06, 1.6, 0.6),
    ("iv4_east",   4.5,  3.5,  0.30, 0.06, 1.6, 0.6),
    ("iv4_top",    3.85, 4.25, 0.30, 1.3,  0.06, 0.6),
    ("ih3",        4.10, 3.6,  0.30, 0.85, 0.06, 0.6),
    ("ih2_left",   0.30, 2.0,  0.30, 2.2,  0.06, 0.6),
    ("ih2_right",  3.50, 2.0,  0.30, 2.2,  0.06, 0.6),
    ("iv2_west",   0.5,  0.5,  0.30, 0.06, 1.6, 0.6),
    ("iv2_east",   1.8,  0.5,  0.30, 0.06, 1.6, 0.6),
    ("iv2_bottom", 1.15, -0.25, 0.30, 1.3, 0.06, 0.6),
    ("iv3_west",   3.2,  0.5,  0.30, 0.06, 1.6, 0.6),
    ("iv3_east",   4.5,  0.5,  0.30, 0.06, 1.6, 0.6),
    ("iv3_bottom", 3.85, -0.25, 0.30, 1.3, 0.06, 0.6),
]


def _box_marker(idx, name, x, y, z, sx, sy, sz, r, g, b):
    m = Marker()
    m.header.frame_id = 'odom'
    m.ns = 'maze'
    m.id = idx
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = float(z)
    m.pose.orientation.w = 1.0
    m.scale.x = float(sx)
    m.scale.y = float(sy)
    m.scale.z = float(sz)
    m.color.r = float(r); m.color.g = float(g); m.color.b = float(b)
    m.color.a = 0.85
    return m


class WorldVisualizer(Node):
    def __init__(self):
        super().__init__('world_visualizer')
        self.declare_parameter('publish_period_s', 1.0)
        period = float(self.get_parameter('publish_period_s').value)
        self.pub = self.create_publisher(MarkerArray, 'world/maze', 10)
        self.create_timer(period, self._publish)
        n = len(_OUTER) + len(_INNER)
        self.get_logger().info(
            f'WorldVisualizer up | {n} walls drawn ({len(_OUTER)} outer + {len(_INNER)} inner)'
        )

    def _publish(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        idx = 0
        for name, x, y, z, sx, sy, sz in _OUTER:
            m = _box_marker(idx, name, x, y, z, sx, sy, sz, 0.65, 0.50, 0.32)
            m.header.stamp = now
            ma.markers.append(m); idx += 1
        for name, x, y, z, sx, sy, sz in _INNER:
            m = _box_marker(idx, name, x, y, z, sx, sy, sz, 0.75, 0.60, 0.40)
            m.header.stamp = now
            ma.markers.append(m); idx += 1
        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = WorldVisualizer()
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
