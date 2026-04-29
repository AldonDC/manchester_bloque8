"""
Nodo: coordinate_transform  (Parte 2 — Árbol de Transformadas)
===============================================================
Publica la transformada dinámica TF entre el frame de odometría y la base del robot.

Responsabilidad:
    Convertir el mensaje de odometría (nav_msgs/Odometry) en una transformada
    dinámica TF2 del tipo:

        {tf_prefix}/odom  →  {tf_prefix}/base_footprint

    Esta transformada es indispensable para que RViz (y cualquier nodo que use
    el árbol de TF) sepa dónde está el robot en el espacio en cada instante.

Por qué tf_prefix es necesario en multi-robot:
    Los frame IDs del árbol de TF son strings globales en ROS 2; NO se aíslan
    automáticamente por namespace. Si dos robots publicaran la misma transformada
    "odom → base_footprint" sin prefijo, el árbol de TF se corrompería porque
    ambos intentarían redefinir el mismo par de frames.
    Con el prefijo, cada robot tiene sus propios frames únicos:
        robot1/odom → robot1/base_footprint
        robot2/odom → robot2/base_footprint

Suscripciones:
    odom  (nav_msgs/Odometry)   — pose actual del robot (de localisation)

Publicaciones TF:
    TransformStamped:  {tf_prefix}/odom  →  {tf_prefix}/base_footprint

Parámetros ROS 2:
    tf_prefix  [str] (default '')  — prefijo de frames para multi-robot
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class CoordinateTransform(Node):
    """Difusor de TF dinámica: odom → base_footprint."""

    def __init__(self):
        super().__init__('coordinate_transform')

        # ── Parámetro de prefijo para aislamiento de frames ───────────────────
        # En escenario de un solo robot se puede dejar en '' (sin prefijo).
        self.declare_parameter('tf_prefix', '')
        prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value

        # Construir los IDs de frame con prefijo si se especificó uno
        self.odom_frame = f'{prefix}/odom'           if prefix else 'odom'
        self.base_frame = f'{prefix}/base_footprint' if prefix else 'base_footprint'

        # ── Difusor de TF dinámica ────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Suscripción a la odometría ────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10)

        self.get_logger().info(
            f'Coordinate Transform iniciado | '
            f'TF: {self.odom_frame} → {self.base_frame}'
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        """Convierte el mensaje Odometry en una transformada TF y la difunde."""

        t = TransformStamped()

        # Encabezado: mismo timestamp que la odometría para coherencia temporal
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.odom_frame   # frame padre  (ej. robot1/odom)
        t.child_frame_id  = self.base_frame   # frame hijo   (ej. robot1/base_footprint)

        # Traslación: posición XY del robot (Z = 0 porque el robot va en plano)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # Rotación: cuaternión copiado directamente de la odometría.
        # Se asignan componente a componente para evitar incompatibilidades
        # de pybind11 al convertir objetos C++ a Python en rclpy.
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Difundir la transformada al árbol global de TF
        self.tf_broadcaster.sendTransform(t)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransform()
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
