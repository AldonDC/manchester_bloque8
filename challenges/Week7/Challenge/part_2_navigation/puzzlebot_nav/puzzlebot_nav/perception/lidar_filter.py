"""
lidar_filter — Temporal smoothing del LiDAR 2D
==============================================
Final Challenge · Part 2 (Sección B opcional)

¿QUÉ HACE?
    Suscribe /scan (raw, ruidoso, salta mucho en RViz) y publica
    /scan_filtered (sensor_msgs/LaserScan) estable y "pro" para
    visualización profesional.

ALGORITMO (sin scipy, solo numpy + stdlib):
    1. Buffer circular de los últimos N scans (N=5 por defecto).
    2. Por cada bin angular, calcula la MEDIANA de los N valores.
         La mediana es robusta a outliers (un rayo espurio NO
         arrastra el promedio como sí lo haría una media).
    3. Rechaza valores fuera de rango (inf, NaN, > max_range) ANTES
         de la mediana, sustituyéndolos por el valor más reciente
         válido (hold).
    4. EMA suave (α=0.4) sobre la mediana resultante para eliminar
         el último jitter sub-pixel sin perder respuesta a obstáculos
         dinámicos. EMA: y_t = α·x_t + (1-α)·y_{t-1}.

QoS:
    Sub: /scan          BEST_EFFORT, depth=1 (latest only)
    Pub: /scan_filtered RELIABLE,    depth=1 (RViz quiere reliable)

¿POR QUÉ ESTO ES "PRO"?
    El scan crudo del LiDAR Gazebo tiene ~3 cm de ruido gaussiano
    por rayo + outliers ocasionales por reflexión múltiple. Si lo
    pintas directo, el patrón "tiembla" — cada frame los puntos
    saltan ±3 cm. Filtrar temporalmente con MEDIANA + EMA elimina
    el temblor visual sin introducir lag perceptible (≤100 ms a 10 Hz).
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LidarFilter(Node):
    """Filtro temporal mediana + EMA sobre /scan."""

    def __init__(self) -> None:
        super().__init__('lidar_filter')

        # Parámetros tuneables desde launch / YAML.
        self.declare_parameter('buffer_size', 5)        # frames en buffer
        self.declare_parameter('ema_alpha',   0.4)      # 0=todo histórico, 1=todo nuevo
        self.declare_parameter('max_range',   10.0)     # m, descarta > esto
        self.declare_parameter('input_topic',  'scan')
        self.declare_parameter('output_topic', 'scan_filtered')

        self.N = int(self.get_parameter('buffer_size').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.max_range = float(self.get_parameter('max_range').value)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Buffer circular de scans (numpy arrays) + estado EMA.
        self._buffer: deque[np.ndarray] = deque(maxlen=self.N)
        self._ema: np.ndarray | None = None
        self._last_valid: np.ndarray | None = None
        self._last_msg: LaserScan | None = None  # geometría del último scan

        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            LaserScan, in_topic, self._on_scan, sub_qos)
        self.pub = self.create_publisher(LaserScan, out_topic, pub_qos)

        # Republish timer a 30 Hz: aunque /scan llegue a 10 Hz, RViz recibe
        # 3 frames del scan suavizado por scan crudo → sensación fluida.
        # No inventa datos: re-emite el EMA con header timestamp actualizado.
        self.declare_parameter('publish_rate_hz', 30.0)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._republish)

        self.get_logger().info(
            f'lidar_filter activo · buffer={self.N} α={self.alpha:.2f} '
            f'max_range={self.max_range} m | {in_topic} → {out_topic} @ {rate} Hz'
        )

    # ─────────────────────────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan) -> None:
        """Callback: actualiza buffer + EMA. La publicación va por timer."""
        ranges = np.asarray(msg.ranges, dtype=np.float32)

        # Paso 1: máscara de validez (finitos, dentro de rango sensor).
        invalid = (~np.isfinite(ranges)
                   | (ranges < msg.range_min)
                   | (ranges > self.max_range))

        # Paso 2: hold-last-valid para huecos (evita inf en la mediana).
        if self._last_valid is None or self._last_valid.shape != ranges.shape:
            self._last_valid = np.where(invalid, self.max_range, ranges).copy()
        else:
            ranges_held = np.where(invalid, self._last_valid, ranges)
            # Actualiza el "último válido" SOLO con lecturas reales.
            self._last_valid = np.where(invalid, self._last_valid, ranges)
            ranges = ranges_held

        # Paso 3: empuja al buffer circular y calcula mediana por bin.
        if self._buffer and self._buffer[0].shape != ranges.shape:
            self._buffer.clear()
            self._ema = None
        self._buffer.append(ranges.copy())

        stack = np.stack(self._buffer, axis=0)          # (N, num_rays)
        median = np.median(stack, axis=0).astype(np.float32)

        # Paso 4: EMA sobre la mediana para suavizar el último jitter.
        if self._ema is None or self._ema.shape != median.shape:
            self._ema = median.copy()
        else:
            self._ema = self.alpha * median + (1.0 - self.alpha) * self._ema

        # Guarda el último mensaje para tener geometría en el republish.
        self._last_msg = msg

    def _republish(self) -> None:
        """Timer @30 Hz: emite el EMA actual con timestamp refrescado."""
        if self._ema is None or self._last_msg is None:
            return
        m = self._last_msg
        out = LaserScan()
        # Timestamp = ahora. Frame_id mantenido. RViz lo trata como dato fresco.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = m.header.frame_id
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        out.ranges = self._ema.astype(np.float32).tolist()
        out.intensities = list(m.intensities)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFilter()
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
