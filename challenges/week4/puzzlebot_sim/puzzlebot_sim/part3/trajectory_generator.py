"""
Nodo: trajectory_set_point_generator  (Parte 3 — Generador de Trayectorias)
============================================================================
Secuencia los waypoints de una figura geométrica y coordina con el controlador.

Responsabilidad:
    Publicar los puntos objetivo (set_points) uno a uno al controlador PID.
    Espera la confirmación de llegada (next_point = True) antes de enviar el
    siguiente waypoint. Además, visualiza las metas como esferas en RViz
    mediante un MarkerArray: verde = objetivo actual, amarillo = pendientes.

Flujo de operación:
    1. Al iniciar, espera 2 segundos antes del primer waypoint (tiempo para
       que el controlador y el simulador estén listos).
    2. Publica el primer waypoint.
    3. Cuando el controlador confirma llegada (next_point = True):
       a. Avanza al siguiente índice en la lista de waypoints (circular).
       b. Espera 2 segundos para estabilizar el robot antes del siguiente punto.
       c. Publica el nuevo waypoint.
    4. Los marcadores de RViz se actualizan cada segundo.

Trayectorias disponibles:
    square   → cuadrado de 1.5 m de lado
    triangle → triángulo con base 1.5 m
    hexagon  → hexágono regular con radio 1.0 m

Suscripciones:
    next_point  (std_msgs/Bool)                — señal de objetivo alcanzado

Publicaciones:
    set_point                  (geometry_msgs/Point)           — waypoint actual
    visualization_marker_array (visualization_msgs/MarkerArray) — esferas en RViz

Parámetros ROS 2:
    shape      [str] (default 'square')  — figura: square | triangle | hexagon
    tf_prefix  [str] (default '')        — prefijo para el frame de los marcadores
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryGenerator(Node):
    """Generador de waypoints secuencial para figuras geométricas."""

    # ── Definición de trayectorias disponibles ────────────────────────────────
    # Cada lista contiene los vértices (x, y) en metros relativos al origen del robot.
    SHAPES = {
        'square': [
            (1.5, 0.0),   # esquina derecha inferior
            (1.5, 1.5),   # esquina derecha superior
            (0.0, 1.5),   # esquina izquierda superior
            (0.0, 0.0),   # origen (cierre del cuadrado)
        ],
        'triangle': [
            (1.5, 0.0),   # vértice derecho de la base
            (0.75, 1.3),  # vértice superior (triángulo casi equilátero)
            (0.0, 0.0),   # origen (cierre del triángulo)
        ],
        'hexagon': [
            (1.0,  0.0),   # vértice 1
            (0.5,  0.87),  # vértice 2
            (-0.5, 0.87),  # vértice 3
            (-1.0, 0.0),   # vértice 4
            (-0.5, -0.87), # vértice 5
            (0.5, -0.87),  # vértice 6  (cierre)
        ],
    }

    def __init__(self):
        super().__init__('trajectory_set_point_generator')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('shape',     'square')
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('x_init',    0.0)
        self.declare_parameter('y_init',    0.0)

        shape  = self.get_parameter('shape').value
        prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        x0     = self.get_parameter('x_init').get_parameter_value().double_value
        y0     = self.get_parameter('y_init').get_parameter_value().double_value

        # Frame en el que se publican los marcadores (debe coincidir con el frame de odom)
        self.odom_frame = f'{prefix}/odom' if prefix else 'odom'

        # ── Selección de trayectoria ──────────────────────────────────────────
        # Si el parámetro 'shape' no coincide con ningún key, usa cuadrado por defecto.
        raw_points = self.SHAPES.get(shape, self.SHAPES['square'])
        
        # Desplazar la figura para que inicie en (x_init, y_init) y no colisione
        self.points = [(px + x0, py + y0) for (px, py) in raw_points]
        
        self.idx    = 0        # índice del waypoint actual
        self.waiting = False   # bandera para ignorar llegadas duplicadas

        # ── Comunicación ROS 2 ────────────────────────────────────────────────
        self.pub_goal    = self.create_publisher(Point,       'set_point',                  10)
        self.pub_markers = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.sub_reached = self.create_subscription(
            Bool, 'next_point', self._reached_callback, 10)

        # Timer de marcadores: actualiza la visualización cada segundo
        self.create_timer(1.0, self._publish_markers)

        # Retardo inicial de 2 s antes del primer waypoint
        # Permite que el controlador y el simulador terminen de inicializarse.
        self.timer_init = self.create_timer(2.0, self._initial_publish)

        self.get_logger().info(
            f'Generador de Trayectorias iniciado | '
            f'figura: {shape.upper()} | {len(self.points)} waypoints | '
            f'frame: {self.odom_frame}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _initial_publish(self):
        """Envía el primer waypoint al arrancar (solo se ejecuta una vez)."""
        self._send_next_point()
        self.timer_init.cancel()   # desactivar el timer de arranque

    def _reached_callback(self, msg: Bool):
        """Avanza al siguiente waypoint cuando el controlador confirma llegada."""
        # La bandera 'waiting' evita avanzar dos veces si llegan mensajes duplicados
        # en el instante en que el robot cruza el umbral de tolerancia.
        if msg.data and not self.waiting:
            self.get_logger().info(
                f'Esquina {self.idx} alcanzada. Esperando 2 s antes del siguiente punto...')

            # Avanzar al siguiente índice de forma circular
            self.idx     = (self.idx + 1) % len(self.points)
            self.waiting = True

            # Timer de un solo disparo: espera 2 s y luego envía el siguiente punto
            self.one_shot = self.create_timer(2.0, self._after_wait)

    def _after_wait(self):
        """Envía el siguiente waypoint tras el retardo entre esquinas."""
        self.one_shot.cancel()   # eliminar el timer para no repetir
        self._send_next_point()

    def _send_next_point(self):
        """Publica el waypoint actual como mensaje Point."""
        x, y   = self.points[self.idx]
        msg    = Point(x=float(x), y=float(y), z=0.0)
        self.pub_goal.publish(msg)
        self.waiting = False
        self.get_logger().info(f'Siguiente objetivo: ({x:.3f}, {y:.3f})')

    # ── Marcadores de RViz ────────────────────────────────────────────────────

    def _publish_markers(self):
        """Publica esferas en RViz para visualizar los waypoints de la trayectoria."""
        ma = MarkerArray()

        for i, (px, py) in enumerate(self.points):
            m = Marker()
            m.header.frame_id = self.odom_frame
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns              = 'waypoints'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD

            m.pose.position.x = float(px)
            m.pose.position.y = float(py)
            m.pose.position.z = 0.02   # ligeramente elevado para no solapar con la grilla

            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08

            # Verde = objetivo actual | Amarillo semitransparente = waypoints pendientes
            if i == self.idx:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.6

            ma.markers.append(m)

        self.pub_markers.publish(ma)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
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
