"""
lidar_processor — Sectorización del LiDAR 2D
=============================================
Mini Challenge · Reactive Navigation (Week 6)

¿QUÉ HACE?
    Lee /scan (sensor_msgs/LaserScan) y publica la distancia mínima en
    tres sectores cardinales: FRONT, LEFT, RIGHT. Estos sectores son el
    "input simplificado" que los algoritmos Bug 0 / Bug 2 consumen.

    En lugar de que cada algoritmo procese 360 rayos cada ciclo, este
    nodo los condensa a 3 escalares (+ un timestamp). Esto desacopla el
    sensor del navegador y deja la lógica de Bug mucho más legible.

SECTORES (relativo al frame del robot — frente = +X = ángulo 0):
        front_half_width = ±30°   (60° totales)
        side_half_width  = ±45°   centrado en ±90°

ROS:
    Subscribe:  /scan      (sensor_msgs/LaserScan)
    Publica:    /bug/d_front   (std_msgs/Float32)
                /bug/d_left    (std_msgs/Float32)
                /bug/d_right   (std_msgs/Float32)

PARÁMETROS:
    front_half_width_deg   half-anchura del cono frontal  (default 30°)
    side_half_width_deg    half-anchura de los conos laterales (default 45°)
    max_valid_range        rangos > este valor se descartan (sensor noise)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarProcessor(Node):
    """Convierte /scan en distancias por sector."""

    def __init__(self):
        super().__init__('lidar_processor')

        self.declare_parameter('front_half_width_deg', 30.0)
        self.declare_parameter('side_half_width_deg',  45.0)
        self.declare_parameter('max_valid_range',      10.0)

        self.front_hw = math.radians(
            self.get_parameter('front_half_width_deg').value)
        self.side_hw = math.radians(
            self.get_parameter('side_half_width_deg').value)
        self.max_range = float(
            self.get_parameter('max_valid_range').value)

        # Publicadores por sector
        self.pub_front = self.create_publisher(Float32, 'bug/d_front', 10)
        self.pub_left  = self.create_publisher(Float32, 'bug/d_left',  10)
        self.pub_right = self.create_publisher(Float32, 'bug/d_right', 10)

        self.sub_scan = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10)

        self.get_logger().info(
            f'Lidar processor activo | front=±{math.degrees(self.front_hw):.0f}° '
            f'side=±{math.degrees(self.side_hw):.0f}° '
            f'max_range={self.max_range} m'
        )

    # ─────────────────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        """Calcula d_front, d_left, d_right desde el arreglo de rangos."""

        # Centros de los 3 sectores en el frame del LiDAR
        front_center =  0.0
        left_center  =  math.pi / 2.0
        right_center = -math.pi / 2.0

        d_front = self._min_in_sector(msg, front_center, self.front_hw)
        d_left  = self._min_in_sector(msg, left_center,  self.side_hw)
        d_right = self._min_in_sector(msg, right_center, self.side_hw)

        self.pub_front.publish(Float32(data=float(d_front)))
        self.pub_left.publish(Float32(data=float(d_left)))
        self.pub_right.publish(Float32(data=float(d_right)))

    # ─────────────────────────────────────────────────────────────────────────

    def _min_in_sector(self, msg: LaserScan, center: float, half_width: float) -> float:
        """
        Devuelve la distancia mínima válida del LaserScan dentro del
        intervalo angular [center - half_width, center + half_width].

        Rangos no válidos (inf, NaN o fuera de [range_min, max_valid_range])
        se descartan. Si no hay rayos válidos, devuelve max_valid_range
        (interpretación: "no obstáculo detectado").
        """
        # Normaliza el centro al intervalo del LaserScan [angle_min, angle_max].
        # En LaserScan típico: angle_min = -π, angle_max = π (lidar 360°).
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return self.max_range

        # Índices de inicio y fin del sector, con wrap-around
        i_center = int(round((self._wrap_angle(center) - msg.angle_min)
                              / msg.angle_increment))
        di = int(round(half_width / msg.angle_increment))

        best = float('inf')
        for k in range(-di, di + 1):
            idx = (i_center + k) % n
            r = msg.ranges[idx]
            if math.isfinite(r) and msg.range_min <= r <= self.max_range:
                if r < best:
                    best = r

        return best if best != float('inf') else self.max_range

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Normaliza un ángulo al rango (-π, π]."""
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
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
