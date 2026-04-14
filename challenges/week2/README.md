# Documentación Técnica: Reto Semana 2 - Integración Cinemática, URDF y Transformadas Explícitas (TF)

**Alumno:** Alfonso 
**Asignatura:** Integración de Robótica y Sistemas Inteligentes 

---

## 1. Introducción al Reto
El objetivo principal de la **Semana 2** consiste en modelar correctamente el robot **Puzzlebot Jetson/Lidar Edition**, establecer la arquitectura jerárquica de sus piezas mediante el árbol de transformadas (TF tree) y dar vida al modelo mediante un simulador cinemático customizado en Python. 

A diferencia del enfoque implícito donde las definiciones fijas preexisten dentro del código puro del URDF, el reto de esta semana nos demanda **iniciar y emitir manualmente las 6 transformadas requeridas por el sistema empleando exclusivamente ROS 2 puro**, asegurando máxima comprensión e injerencia sobre el árbol estructural.

---

## 2. Red Topológica: Nodos, Tópicos y Comunicación

El flujo de información en nuestra red interconecta tres nodos centrales, logrando que un "esqueleto" inerte en URDF se traduzca fluidamente en movimiento tridimensional espacial. Para comprenderlo mejor, a continuación se detallan exactamente en qué archivos y líneas de código "nacen" estas comunicaciones.

### Despliegue de Nodos (Dónde se instancian)
El ecosistema completo se levanta desde el archivo **`puzzlebot_launch.py`**. Aquí declaramos y arrancamos los procesos:

1.  **`robot_state_publisher` (Nodo Oficial):** Se encarga de procesar el URDF.
    *   *Ubicación en código (`puzzlebot_launch.py:L37`)*:
        ```python
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
        )
        ```
2.  **`puzzlebot_kinematic_sim` (Nuestro Nodo Python):** Es nuestra clase de Python compilada.
    *   *Ubicación en código (`puzzlebot_launch.py:L43`)*:
        ```python
        Node(
            package="puzzlebot_sim",
            executable="joint_state_publisher",
            name="puzzlebot_kinematic_sim",
        )
        ```
3.  **`rviz2` (Nodo Gráfico):** Observa a los dos anteriores y renderiza.
    *   *Ubicación en código (`puzzlebot_launch.py:L49`)*.

### Vías de Comunicación (Tópicos)
Nuestra clase maestra dentro de **`joint_state_publisher.py`** es el corazón emisor. Utiliza dos tópicos principales:

*   **Tópico `/tf` (Transformadas Especiales)**: Es el canal que dicta dónde están las extremidades y el chasis en el mundo 3D (`geometry_msgs/msg/TransformStamped`).
    *   *Creación del canal (`joint_state_publisher.py:L31`)*:
        ```python
        self.tf_broadcaster = TransformBroadcaster(self)
        ```
    *   *Despacho de datos (`joint_state_publisher.py:L53`)*:
        ```python
        self.tf_broadcaster.sendTransform(t) # Llamado 6 veces por cada ciclo
        ```

*   **Tópico `/joint_states` (Giro Angular Físico)**: Le avisa a `robot_state_publisher` cuánto rodamiento (en radianes) tienen las mallas de las ruedas para animar el render.
    *   *Creación del canal (`joint_state_publisher.py:L34`)*:
        ```python
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        ```
    *   *Despacho de datos (`joint_state_publisher.py:L91`)*:
        ```python
        js = JointState()
        js.name = ["wheel_l_joint", "wheel_r_joint"]
        js.position = [self.wheel_angle, self.wheel_angle]
        self.joint_pub.publish(js) # Susurrado cada ciclo de reloj
        ```

---

## 3. Paseo de Código: Lanzamiento y Orquestación

### `puzzlebot_launch.py` (El Director)
El *Launch File* sirve para centralizar todos los nodos dispersos de las terminales. Despliega la red global e inyecta dependencias al mismo tiempo.

**Lo qué sucede paso a paso en el Launch:**
1.  Abre y lee localmente nuestro archivo `puzzlebot.urdf`. Convierte las instrucciones abstractas `package://` en una ruta cruda manejable para inyectárselo a nuestro robot_state_publisher mediante `parameters=[{"robot_description": robot_desc}]`.
2.  Dado que asumiremos todo el control en el script final (ver sección 4), **fueron suprimidos nodos estáticos**, como el clásico `static_transform_publisher` que vinculaba `map -> odom`. Ahora, TODO residirá algorítmicamente en nuestro archivo temporal.
3.  Inicia la herramienta visual nativa `rviz2` bajo nuestra configuración preestablecida (`.rviz`), y ejecuta el script final customizado `joint_state_publisher.py` de nuestro paquete.

---

## 4. Paseo de Código: Simulador Pitónico y Geometría 

### `joint_state_publisher.py` (El Cerebro Cinemático)
Aquí es donde ocurre la magia pura del reto. Diseñamos un Nodo custom (`PuzzlebotKinematicSim`) centrado en publicar rigurosamente los 6 ejes demandados sin inferir ayuda indirecta.

#### 4.1. Setup y Arquitectura Interna de Clase
En su función inicial `__init__`, declaramos las variables de movimiento (velocidad lineal `v` de 0.15 m/s, velocidad angular `w` de 0.35 rad/s) y nuestro tiempo central: el bucle será emitido con una cadencia temporal altísima a **50Hz (`dt = 0.02s`)** activada por un "Timer".

Además, preparamos dos canales de despacho para el robot: 
* Un `TransformBroadcaster(self)` que despacha frames en 3D.
* Una central de radiodifusión `joint_pub` encasillada exclusivamente al tópico estricto `/joint_states`.

#### 4.2. El Bucle Real y Computación Computacional (`timer_callback`)
La primera fase dentro de la ventana del callback se encarga de aplicar los lineamientos de cálculo en cinemática diferencial simple (euleriana):
```python
self.th += self.w * self.dt # Integrar theta global
self.x += self.v * math.cos(self.th) * self.dt
self.y += self.v * math.sin(self.th) * self.dt
self.wheel_angle += (self.v / 0.05) * self.dt
```
Una vez lograda la reubicación actual, pasamos a generar en paralelo la cascada estructural visual explícita requerida:

#### 4.3. Las 6 Transformadas Explícitas (TF Tree)
Para satisfacer el objetivo, generamos manualmente a través del `self.publish_tf()` cada una de las 6 derivaciones (Con sus valores oficiales 2026 pre-cargados al archivo base):
1.  **`map` ➔ `odom`**: (Estática Identidad). Eje raíz mundial principal.
2.  **`odom` ➔ `base_footprint`**: (Dinámica Integrada). Posiciona globalmente a nuestro robot en su `x`, `y` y rotación Yaw respecto al inicio global.
3.  **`base_footprint` ➔ `base_link`**: Refleja la elevación del chasis por arriba del tapete `Z = 0.05`.
4.  **`base_link` ➔ `caster`**: Coloca el pivote esférico hacia atrás de los ejes motrices `[-0.095, 0.0, -0.03]`.
5.  **`base_link` ➔ `wheel_l`**: La rueda izquierda anclada a su lado con rotación progresiva orientada.
6.  **`base_link` ➔ `wheel_r`**: La rueda derecha. (¡El anclaje de este eje implicó un reto matemático explicito profundo!).

---

## 5. El Desafío Ingenieril: Matemática Orientada 

### La Rueda Derecha Invertida y Cuaterniones (`wheel_r`)
Las lógicas visualizan correctamente el lado izquierdo, pero para armar físicamente el rompecabezas a la derecha los diseñadores ensamblan el segundo motor 180° grados volteado, requiriendo que un cuaternión efectúe el mismo Yaw constante (180 Grados) mientras integra el pitch progresivamente.

Originalmente el paquete "transforms3d" y dependencias profundas como "NumPy 2.x" presentaban enormes choques visuales de integración inestables frente a librerías modernas. ¿Nuestra Solución Integral? **Computarizar el cuaternión en Python base mediante producto directo**.

*  Si la rotación base obligada es **Yaw de 180° ($\pi$)**, su Cuaternión Puro (eje Z global) es: $q_{base}=(w=0, x=0, y=0, z=1)$.
*  Si nuestro rodamiento progresivamente deseado es un simple **Pitch local ($\alpha$)**, este es igual al Cuaternión: $q_{rodamiento}=(w=\cos(\alpha/2), x=0, y=\sin(\alpha/2), z=0)$.

Ejecutamos el producto final sin usar bibliotecas NumPy y logramos la matriz unificada simplificada exacta extraíble para código nativo:
```python
# Componentes puros finales del producto base implementados 
s = math.sin(self.wheel_angle / 2.0)
c = math.cos(self.wheel_angle / 2.0)

self.publish_tf("base_link", "wheel_r", 
                x=0.052, y=-0.095, z=-0.0025,
                qx=s, qy=0.0, qz=c, qw=0.0)
```
Resultado garantizado: El elemento `wheel_r` no sólo respeta la colisión, sino que gira a la par y de excelente manera con la rueda izquierda gracias a la negación invertida indirecta del producto puro.

---

## 6. Procedimientos de Ejecución al Sistema

Para visualizar y comprobar la integralidad estructural del esquema diseñado aquí documentado, por favor realice los siguientes comandos bajo la terminal de entorno `Humble`:

```bash
#  1. Estructura y compilado nativo (Build):
colcon build --packages-select puzzlebot_sim

#  2. Precarga a variables globales e Inicialización:
source install/setup.bash
ros2 launch puzzlebot_sim puzzlebot_launch.py
```
*(Opcional: Esperar 5 Segundos post lanzamiento durante el arranque para consultar un comprobante estáticamente imprimido del árbol "Frame" validado, alojado de manera interna como `challenges/output_pdf/frames.pdf`).*
