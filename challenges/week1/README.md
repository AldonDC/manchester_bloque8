# Reto Semana 1: Modelado y Cinematica de Puzzlebot

## Objetivo del Proyecto
El presente proyecto tiene como finalidad el modelado tridimensional y la implementacion cinematica del robot movil Puzzlebot Jetson/Lidar Edition en el entorno ROS 2 Humble. Se busca establecer una base solida para la navegacion mediante la correcta configuracion del arbol de transformadas (TF) y la simulacion de movimiento en bucle abierto.

## Arquitectura de Transformadas (TF)
Para lograr el movimiento y la representacion jerarquica del robot, se configuraron un total de **6 transformadas** fundamentales:

1. **map ➔ odom** (Estatica): Define el origen global del sistema. Se especifica en el archivo `puzzlebot_launch.py` mediante un `static_transform_publisher`.
2. **odom ➔ base_footprint** (Dinamica): Representa el desplazamiento del robot en el mundo. Esta es calculada por el nodo `joint_state_publisher.py` integrando las velocidades lineal y angular.
3. **base_footprint ➔ base_link** (Fija): Define la altura del chasis respecto al suelo (0.05m en Z). Se encuentra definida en el `puzzlebot.urdf`.
4. **base_link ➔ wheel_l** (Continua): Union de la rueda izquierda. Su giro visual es controlado mediante mensajes de tipo `JointState`.
5. **base_link ➔ wheel_r** (Continua): Union de la rueda derecha. Al igual que la izquierda, su rotacion depende del topico `/joint_states`.
6. **base_link ➔ caster** (Fija): Union de la rueda loca trasera.

## Detalles de Implementacion en Codigo

### 1. Publicacion de Transformada Dinamica
En el script `joint_state_publisher.py`, la posicion se publica dentro del `timer_callback`. El fragmento relevante es:
```python
t = TransformStamped()
t.header.frame_id = "odom"
t.child_frame_id = "base_footprint"
t.transform.translation.x = self.x
t.transform.translation.y = self.y
# ... conversion de angulo a cuaternion ...
self.tf_broadcaster.sendTransform(t)
```

### 2. Control Individual de Articulaciones (Joint States)
Para que las ruedas giren de forma independiente en RViz, el nodo publica en el topico `/joint_states`. Se especifican los nombres exactos definidos en el URDF:
```python
js = JointState()
js.name = ["wheel_l_joint", "wheel_r_joint"]
js.position = [self.wheel_angle, self.wheel_angle]
self.joint_pub.publish(js)
```
*Dato tecnico:* Gracias a que en el URDF espejeamos el eje de la rueda derecha (`axis="0 -1 0"`), ambas ruedas giran hacia adelante simultaneamente al recibir el mismo valor de posicion.

## Instrucciones de Uso

### 1. Compilacion
```bash
rm -rf build install log
colcon build --packages-select puzzlebot_sim
source install/setup.bash
```

### 2. Ejecucion y Verificacion
```bash
ros2 launch puzzlebot_sim puzzlebot_launch.py
```
Tras 5 segundos, se generara el archivo **`frames.pdf`**. Al abrirlo, el diagrama debe mostrar la cascada completa desde `map` hasta los links de las ruedas, validando asi el cumplimiento de los requerimientos de modelado.
