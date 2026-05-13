"""
Nodo: open_loop_driver  —  Helper para Task 2 (calibración)
============================================================
Mini Challenge 4 · Manchester Robotics

¿POR QUÉ EXISTE ESTE NODO?
    El Task 2 del PDF pide hacer experimentos REPETIBLES para tunear
    k_r y k_l. Si usáramos el controlador PID las dinámicas del lazo
    cerrado contaminarían la medición — necesitamos una entrada
    determinística y conocida.

    Solución: inyectar cmd_vel constante por un tiempo fijo, SIN
    realimentación. Así la única fuente de error es el ruido de las
    ruedas, que es exactamente lo que queremos medir.

EXPERIMENTOS DEL PDF QUE SOPORTA:
    1) Recta de 1 m   →  v = 0.15 m/s, ω = 0,  duración 6.67 s
    2) Rotación 1 vuelta → v = 0, ω = π/2 rad/s, duración 4 s

ROS:
    Subscribe:  (ninguno — es open-loop por definición)
    Publica:    cmd_vel  (geometry_msgs/Twist)  → al Real/Sim Robot

PARÁMETROS:
    linear_velocity   [m/s]   v constante a inyectar
    angular_velocity  [rad/s] ω constante a inyectar
    duration          [s]     tiempo total de excitación; después → 0
    publish_rate      [Hz]    frecuencia de publicación de cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoopDriver(Node):
    """Inyecta un comando de velocidad constante por un tiempo finito."""

    def __init__(self):
        super().__init__('open_loop_driver')

        self.declare_parameter('linear_velocity',  0.15)
        self.declare_parameter('angular_velocity', 0.0)
        self.declare_parameter('duration',         6.67)
        self.declare_parameter('publish_rate',     50.0)

        self.v  = self.get_parameter('linear_velocity').value
        self.w  = self.get_parameter('angular_velocity').value
        self.T  = self.get_parameter('duration').value
        rate    = self.get_parameter('publish_rate').value
        self.dt = 1.0 / float(rate)

        self.elapsed  = 0.0
        self.finished = False

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer   = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'Open-loop driver | v={self.v} m/s  ω={self.w} rad/s  '
            f'duración={self.T} s'
        )

    def _tick(self):
        """Lazo periódico: durante T segundos publica (v, ω); después, cero."""
        cmd = Twist()

        if self.elapsed < self.T:
            # INICIO: el robot recibe el comando constante.
            cmd.linear.x  = float(self.v)
            cmd.angular.z = float(self.w)
            self.elapsed += self.dt
        else:
            # FINAL: comando cero para detener al robot.
            # No matamos el nodo: seguimos publicando 0 para que el RViz
            # mantenga la última pose mientras observamos la elipse final.
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            if not self.finished:
                self.finished = True
                self.get_logger().info(
                    'Experimento terminado. Robot detenido. '
                    'Observa la elipse final en RViz.'
                )

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopDriver()
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
