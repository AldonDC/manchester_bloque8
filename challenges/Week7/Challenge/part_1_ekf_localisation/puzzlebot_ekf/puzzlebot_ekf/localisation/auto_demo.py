"""
auto_demo — Drives the robot through the 3 scenarios the PDF asks for.

Scenarios demonstrated (in this order):

    [1] multi-marker observation
        Robot is still. Camera sees M0 + M3 simultaneously -> two
        corrections per frame -> covariance ellipse stays tight.

    [2] motion with continuous observation
        Robot moves forward briefly. Range to markers shrinks; the
        ellipse stays small.

    [3] no-marker observation
        Robot yaws toward a wall until no marker is visible. Only the
        predict step runs; covariance grows visibly in RViz.

    [4] re-localisation (partial / single marker observation)
        Robot yaws back and a single marker reappears. Covariance
        collapses immediately -> the ellipse shrinks again.

This node is launched together with the EKF stack so the entire demo
runs from one terminal. It starts after a fixed warm-up so the sim,
the bridge, the camera, the EKF and RViz are all running.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# Each step: (phase_id, description, linear.x, angular.z, duration_seconds)
# El phase_id se publica en /demo/phase para que ekf_node agrupe las métricas
# (RMSE, NEES, updates aceptados/rechazados) por escenario del PDF.
# Rotation-only plan: safer (no wall collisions), demonstrates the same
# 3 PDF scenarios visually clearly.
_DEMO_PLAN = [
    ('multi_marker',   "[1/4] Quieto — multi-marker visible · ELIPSE TIGHT",    0.0,  0.0, 6.0),
    ('scanning',       "[2/4] Girar 180° lento — barre todos los markers",     0.0,  0.35, 9.0),
    ('no_marker',      "[3/4] QUIETO mirando al muro — sin markers · P CRECE", 0.0,  0.0, 8.0),
    ('reacquire',      "[4/4] Girar de vuelta — markers reaparecen · P ENCOGE",0.0, -0.35, 9.0),
    ('final_idle',     "       Quieto final 4 s",                              0.0,  0.0, 4.0),
]


class AutoDemo(Node):

    def __init__(self):
        super().__init__('auto_demo')

        self.declare_parameter('warmup_s', 10.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.warmup = float(self.get_parameter('warmup_s').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # /demo/phase: ekf_node se suscribe para segmentar las métricas por
        # escenario del PDF (multi-marker / sin marker / reaparición / etc.).
        self.pub_phase = self.create_publisher(String, 'demo/phase', 10)

        self.get_logger().info(
            f'AutoDemo armed | warmup={self.warmup:.1f}s | rate={self.rate_hz:.0f} Hz'
        )
        self.get_logger().info(
            'Will start the demo plan after the warm-up.'
        )

    def run_plan(self):
        # Initial pause so Gazebo + bridge + EKF + RViz are all up.
        self.get_logger().info(
            f'Waiting {self.warmup:.0f} s for the rest of the stack to settle...'
        )
        deadline = time.time() + self.warmup
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        period = 1.0 / self.rate_hz
        for phase_id, desc, lx, az, dur in _DEMO_PLAN:
            self.get_logger().info(desc)
            # Publica la fase ANTES de empezar, varias veces para que el ekf
            # node la reciba aunque haya jitter de latched/qos.
            for _ in range(3):
                self.pub_phase.publish(String(data=phase_id))
                time.sleep(0.02)
            step_deadline = time.time() + dur
            msg = Twist()
            msg.linear.x = float(lx)
            msg.angular.z = float(az)
            while time.time() < step_deadline:
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=period)

        # Señal de fin para que ekf_node cierre la última fase y vuelque el
        # reporte agregado por fases en el siguiente shutdown.
        self.pub_phase.publish(String(data='__end__'))
        # Explicit stop at the end.
        self.pub.publish(Twist())
        self.get_logger().info(
            'Demo complete. RViz recording should show: multi-marker '
            'correction, ellipse growth (no markers), and re-localisation.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoDemo()
    try:
        node.run_plan()
    except KeyboardInterrupt:
        pass
    finally:
        # Last-ditch stop, just in case.
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
