"""
Nodo: coordinate_transform  —  Difusor de TF dinámica
======================================================
Mini Challenge 4 · Manchester Robotics

¿POR QUÉ EXISTE ESTE NODO?
    El PDF dice: "the student must define the transformation to be used".
    RViz puede dibujar la elipse Σ_k del odometry message PERO necesita
    saber DÓNDE colocarla. Eso lo logra consultando el árbol de TF.

    Este nodo lee /odom y publica la transformación TF correspondiente:
        odom (frame padre)  →  base_footprint (frame del robot)
    para que RViz coloque el modelo 3D y la elipse en el lugar correcto.

DATAFLOW:
    /odom (de Localisation)  ──►  tf_broadcaster  ──►  /tf (escucha RViz)

NOTA sobre tf_prefix (multi-robot):
    En ROS 2 los frame IDs son globales (no se aíslan por namespace).
    Si hubiera dos robots, ambos publicando "odom → base_footprint",
    se pisarían entre sí. Con prefijo cada robot tiene su propio par:
        robot1/odom → robot1/base_footprint
        robot2/odom → robot2/base_footprint

ROS:
    Subscribe:  odom (nav_msgs/Odometry)        — pose de Localisation
    Broadcast:  /tf  (geometry_msgs/TransformStamped)
                    odom → base_footprint
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
        """
        Pipeline:  /odom (pose + cov)  ──►  TransformStamped  ──►  /tf
        """
        t = TransformStamped()

        # Sincronizar timestamps: usar el de /odom para que RViz alinee
        # la TF con el mensaje de covarianza del mismo ciclo.
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.odom_frame   # padre (ej. robot1/odom)
        t.child_frame_id  = self.base_frame   # hijo  (ej. robot1/base_footprint)

        # TRASLACIÓN: copiada de la pose. z=0 porque el Puzzlebot es 2D.
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # ROTACIÓN: cuaternión copiado componente a componente.
        # (Evitar `t.transform.rotation = msg...orientation`: rclpy/pybind11
        # a veces falla al asignar el objeto C++ completo.)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Difundir al árbol global de TF — lo escucha RViz y cualquier nodo.
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
