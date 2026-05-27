"""
waypoint_manager — Closed-loop multi-waypoint sequencer for Part 2.

Reads a list of waypoints from parameters, publishes one at a time to /goal,
and advances to the next when the Bug node reports 'GOAL_REACHED' on
/bug/state. After the last waypoint, it loops back to the first to close
the trajectory, satisfying the Final Challenge requirement of a closed
trajectory with >=4 target points.

Contract:
    Publishes:  /goal                       (geometry_msgs/Point)
                /visualization_marker_array (visualization_msgs/MarkerArray)
                /waypoint_manager/state     (std_msgs/String)
    Subscribes: /bug/state                  (std_msgs/String)

Parameters:
    waypoints       flat list [x0,y0, x1,y1, ...] in metres (>= 8 entries -> 4 pts)
    frame_id        marker frame                                  (default 'odom')
    loop            close the trajectory by cycling back to wp 0  (default True)
    dwell_time      seconds to wait after reaching a wp           (default 1.0)
    publish_period  re-publish current goal every N s             (default 1.0)

Why a dedicated manager (not just chaining inside Bug):
    Keeps the Bug nodes single-goal, which is how the Mini Challenge had them.
    The manager is a thin sequencer above Bug, decoupled and testable on its
    own. Same Bug binary handles one or many waypoints.
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class _State(Enum):
    RUNNING = 'RUNNING'
    DWELL = 'DWELL'
    FINISHED = 'FINISHED'


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('waypoints',
                               [2.0, 0.0,
                                2.0, 2.0,
                                0.0, 2.0,
                                -1.5, 1.0])
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('loop', True)
        self.declare_parameter('dwell_time', 1.0)
        self.declare_parameter('publish_period', 1.0)

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) < 8 or len(flat) % 2 != 0:
            raise ValueError(
                f'waypoints must be a flat list of >=8 even-length floats '
                f'(>=4 waypoints). Got {len(flat)} entries.'
            )
        self.waypoints = [(float(flat[i]), float(flat[i + 1]))
                          for i in range(0, len(flat), 2)]

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.loop = bool(self.get_parameter('loop').value)
        self.dwell_time = float(self.get_parameter('dwell_time').value)
        period = float(self.get_parameter('publish_period').value)

        self.idx = 0
        self.state = _State.RUNNING
        self.dwell_started_at = None

        self.pub_goal = self.create_publisher(Point, 'goal', 10)
        self.pub_state = self.create_publisher(String, 'waypoint_manager/state', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, 'visualization_marker_array', 10)

        self.create_subscription(String, 'bug/state', self._bug_state_cb, 10)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'WaypointManager | {len(self.waypoints)} waypoints, '
            f'loop={self.loop}, dwell={self.dwell_time:.1f}s'
        )

    def _bug_state_cb(self, msg: String):
        if self.state != _State.RUNNING:
            return
        if msg.data == 'GOAL_REACHED':
            self.get_logger().info(
                f'wp[{self.idx}] reached @ {self.waypoints[self.idx]}'
            )
            self.state = _State.DWELL
            self.dwell_started_at = self.get_clock().now()

    def _advance(self):
        self.idx += 1
        if self.idx >= len(self.waypoints):
            if self.loop:
                self.idx = 0
                self.get_logger().info('closing trajectory — back to wp[0]')
            else:
                self.state = _State.FINISHED
                self.get_logger().info('trajectory complete (loop=False)')
                return
        self.state = _State.RUNNING

    def _tick(self):
        now = self.get_clock().now()

        if self.state == _State.DWELL:
            elapsed = (now - self.dwell_started_at).nanoseconds * 1e-9
            if elapsed >= self.dwell_time:
                self._advance()

        self._publish_current_goal()
        self._publish_markers()
        self._publish_status()

    def _publish_current_goal(self):
        if self.state == _State.FINISHED:
            return
        x, y = self.waypoints[self.idx]
        self.pub_goal.publish(Point(x=x, y=y, z=0.0))

    def _publish_status(self):
        tag = f'{self.state.value} wp[{self.idx}]'
        self.pub_state.publish(String(data=tag))

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = m.scale.z = 0.18
            if i == self.idx and self.state != _State.FINISHED:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.6, 1.0, 0.8
            ma.markers.append(m)

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = 'waypoints'
        line.id = 1000
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 0.6, 1.0, 0.6
        pts = list(self.waypoints) + ([self.waypoints[0]] if self.loop else [])
        for x, y in pts:
            p = Point()
            p.x, p.y, p.z = x, y, 0.02
            line.points.append(p)
        ma.markers.append(line)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
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
