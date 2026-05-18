# Mini Challenge — Reactive Navigation (Bug 0 / Bug 2)
### Week 6 · Manchester Robotics × NVIDIA

> Autor: **Alfonso Diaz** · Curso: Robótica móvil · Plataforma: **Puzzlebot Jetson Lidar Edition** sobre **Gazebo Harmonic**
> ROS 2 Humble · Python · NumPy puro (sin librerías externas)

---

## Tabla de contenidos

1. [Qué pide el reto](#1-qué-pide-el-reto)
2. [Vista de pájaro: cómo se mueve el robot](#2-vista-de-pájaro-cómo-se-mueve-el-robot)
3. [Teoría y matemática](#3-teoría-y-matemática)
4. [Arquitectura del sistema (topics y dataflow)](#4-arquitectura-del-sistema-topics-y-dataflow)
5. [Catálogo de nodos](#5-catálogo-de-nodos)
6. [Catálogo de topics y mensajes](#6-catálogo-de-topics-y-mensajes)
7. [Máquinas de estado implementadas](#7-máquinas-de-estado-implementadas)
8. [Explicación a fondo de cada código](#8-explicación-a-fondo-de-cada-código)
9. [Launch files](#9-launch-files)
10. [Estructura del paquete](#10-estructura-del-paquete)
11. [Cómo correrlo](#11-cómo-correrlo)
12. [Mundos disponibles](#12-mundos-disponibles)
13. [Tabla de parámetros](#13-tabla-de-parámetros)
14. [Cumplimiento del enunciado](#14-cumplimiento-del-enunciado)
15. [Mejoras justificadas](#15-mejoras-justificadas)
16. [Troubleshooting](#16-troubleshooting)

---

## 1. ¿Qué pide el reto?

Construir un **navegador reactivo** para el Puzzlebot que, usando el LiDAR 2D del simulador MCR2, llegue desde una pose inicial hasta un objetivo $(x_T, y_T)$ sorteando obstáculos. Hay dos tareas:

| Task | Algoritmo | Idea |
|------|-----------|------|
| **Task 1** | **Bug 0** | Apunta al goal y avanza. Si hay obstáculo, bordéalo hasta que la línea de vista al goal esté libre, y entonces sigue. |
| **Task 2** | **Bug 2** | Igual, pero la condición de liberación es volver a tocar la *m-line* (línea recta start→goal) más cerca del objetivo. |

### Reglas (del PDF)

- Reutilizar el **multi-point position controller** del Mini Challenge 2 (`controller.py`).
- Considerar la **covarianza creciente** $\Sigma_k$ alrededor de la pose (Reto 4).
- Probar en **varios mundos** de Gazebo.
- Definir un **sampling time** correcto.
- Sólo **NumPy + stdlib** (sin scipy, sin transforms3d).
- Sin teleop ni intervención humana en runtime.

---

## 2. Vista de pájaro: cómo se mueve el robot

Antes de las matemáticas, **cómo viaja la información** dentro del sistema, de izquierda a derecha:

```
   GAZEBO                ROS 2                                      GAZEBO
  ┌──────┐         ┌─────────────────────────────────────┐         ┌──────┐
  │      │ /scan   │  lidar_processor                    │         │      │
  │ LiDAR├────────►│  ──► /bug/d_front, d_left, d_right  │         │      │
  │      │         │                                     │         │      │
  │      │ /joint  │  wheel_state_bridge                 │         │      │
  │  ω_r ├────────►│  ──► /wr, /wl                       │         │      │
  │  ω_l │ _states │                                     │         │ Diff │
  │      │         │  localisation                       │         │Drive │
  │      │         │  ──► /odom (pose + Σ_k)             │         │ Plug │
  │      │         │                                     │         │ in   │
  │      │         │  coordinate_transform               │         │      │
  │      │         │  ──► /tf  (odom → base_footprint)   │         │      │
  │      │         │                                     │         │      │
  │      │         │  goal_publisher                     │         │      │
  │      │         │  ──► /goal  (x_T, y_T fijo)         │         │      │
  │      │         │                                     │         │      │
  │      │         │  bug0 / bug2                        │         │      │
  │      │         │  ──► /set_point  (waypoint dinámico)│         │      │
  │      │         │  ──► /bug/state, /bug/markers       │         │      │
  │      │         │                                     │         │      │
  │      │/cmd_vel │  controller (PID del Reto 2)        │         │      │
  │   v,ω│◄────────│  ──► /cmd_vel                       │         │      │
  └──────┘         └─────────────────────────────────────┘         └──────┘
```

**En palabras:**

1. **Gazebo simula** el LiDAR y la odometría de las ruedas y los publica en `/scan` y `/joint_states`.
2. **`lidar_processor`** colapsa los 360 rayos del LiDAR a 3 escalares: `d_front`, `d_left`, `d_right` (distancia mínima en cada sector cardinal).
3. **`wheel_state_bridge`** convierte `/joint_states` (un mensaje con todas las juntas) en dos topics individuales `/wr` y `/wl` (velocidades de rueda separadas), que es lo que espera el localisation del Reto 4.
4. **`localisation`** integra dead-reckoning con esas velocidades para estimar la pose $(x, y, \theta)$ y propaga la matriz de covarianza $\Sigma_k$ siguiendo el modelo del Reto 4. Publica `/odom` con la pose **y** la covarianza embebida.
5. **`coordinate_transform`** convierte cada mensaje `/odom` en un TF dinámico `odom → base_footprint` para que RViz pueda dibujar el robot y la elipse de covarianza.
6. **`goal_publisher`** publica una sola vez por segundo el goal final $(x_T, y_T)$ en `/goal`.
7. **`bug0`** o **`bug2`** es el "cerebro": consume `/odom` + los `/bug/d_*` + `/goal`, ejecuta la FSM Bug, y emite un **waypoint dinámico** en `/set_point`. Este waypoint NO es el goal final: es el siguiente punto que quiere que el robot persiga (puede ser el goal mismo, o un punto adelante del robot sesgado lateralmente cuando está bordeando una pared).
8. **`controller`** (el PID del Reto 2) recibe `/set_point` y `/odom`, calcula $(v, \omega)$ y los publica en `/cmd_vel`.
9. **Gazebo** consume `/cmd_vel` a través de su plugin DiffDrive y mueve las ruedas. El ciclo se cierra.

**Importante:** el nodo Bug **no manda `/cmd_vel` directo**. Manda un punto `(x, y)` al PID del Reto 2, que es quien finalmente le habla a las ruedas. Esto cumple literalmente el requisito del PDF de reutilizar el controller del Mini Challenge 2.

---

## 3. Teoría y matemática

### 3.1 Bug 0

Algoritmo más simple de la familia Bug (Choset et al., *Principles of Robot Motion*, 2005):

```
mientras |pose − goal| > tol:
    si NO hay obstáculo enfrente:
        moverse en línea recta hacia el goal
    si hay obstáculo enfrente:
        bordearlo (wall-following) hasta que la dirección al goal
        esté libre, entonces volver a "ir al goal"
```

Es greedy: **no recuerda nada del obstáculo**. Funciona perfecto en entornos convexos y en cascadas razonables. En formas cóncavas tipo "U" puede dar vueltas, por eso existe Bug 2.

### 3.2 Bug 2

Mejora propuesta por Lumelsky & Stepanov (1986). En lugar de liberar el bordeado cuando el frente esté libre, se libera cuando el robot vuelve a cruzar la **m-line** — la línea recta imaginaria de start a goal — Y está más cerca del goal que cuando entró al obstáculo (`hit_point`):

```
m-line ← recta(start, goal)
hit_point ← None

mientras |pose − goal| > tol:
    si en GO_TO_GOAL:
        si NO hay obstáculo enfrente → avanzar al goal
        si hay obstáculo → registrar hit_point, entrar a WALL_FOLLOW
    si en WALL_FOLLOW:
        bordear; salir SOLO cuando:
            |pose − m_line| < ε      Y
            |pose − goal|  < |hit_point − goal|  Y
            d_front > clear_distance
```

#### 3.2.1 Distancia de un punto a la m-line

Si la m-line va de $\mathbf{A} = (x_s, y_s)$ a $\mathbf{B} = (x_T, y_T)$, la distancia perpendicular de un punto $P = (x, y)$ a la recta $AB$ es:

$$d_{m\text{-line}}(P) = \frac{|(x_T - x_s)(y_s - y) - (x_s - x)(y_T - y_s)|}{\sqrt{(x_T - x_s)^2 + (y_T - y_s)^2}}$$

Implementado tal cual en `_distance_to_mline(x, y)` de `bug2.py`.

### 3.3 Modelo cinemático del Puzzlebot

El Puzzlebot es un robot diferencial de 2 ruedas. Su entrada son las velocidades angulares de cada rueda $(\omega_r, \omega_l)$ y su salida es la velocidad lineal del centro $v$ y la angular del cuerpo $\omega$:

$$v = \frac{r \cdot (\omega_r + \omega_l)}{2} \qquad \omega = \frac{r \cdot (\omega_r - \omega_l)}{L}$$

donde $r = 0.05$ m es el radio de la rueda y $L = 0.19$ m la distancia entre ruedas. La pose se integra por Euler a 50 Hz en `localisation.py`:

$$\begin{aligned}
x_{k+1} &= x_k + v \cos(\theta_k) \cdot \Delta t \\
y_{k+1} &= y_k + v \sin(\theta_k) \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega \cdot \Delta t
\end{aligned}$$

### 3.4 Propagación de covarianza $\Sigma_k$ (Reto 4)

La covarianza de la pose crece a cada paso porque no hay observaciones que la corrijan:

$$\Sigma_k = H_k \, \Sigma_{k-1} \, H_k^\top + Q_k$$

donde:

- $H_k$ es el Jacobiano del modelo respecto a la pose previa:

$$H_k = \begin{bmatrix} 1 & 0 & -\Delta t \cdot v \sin\theta \\ 0 & 1 & \phantom{-}\Delta t \cdot v \cos\theta \\ 0 & 0 & 1 \end{bmatrix}$$

- $\nabla\omega_k$ es el Jacobiano respecto a las entradas $(\omega_r, \omega_l)$:

$$\nabla\omega_k = \frac{r \cdot \Delta t}{2} \begin{bmatrix} \cos\theta & \cos\theta \\ \sin\theta & \sin\theta \\ 2/L & -2/L \end{bmatrix}$$

- $\Sigma_{\Delta,k}$ es el ruido del encoder, proporcional a la magnitud de velocidad de cada rueda:

$$\Sigma_{\Delta,k} = \begin{bmatrix} k_r |\omega_r| & 0 \\ 0 & k_l |\omega_l| \end{bmatrix}$$

- $Q_k = \nabla\omega_k \cdot \Sigma_{\Delta,k} \cdot \nabla\omega_k^\top$ propaga el ruido al espacio de la pose.

Los ganancias $k_r = k_l = 0.05$ son calibrables. Esta matriz se mapea a `pose.covariance[36]` del mensaje `nav_msgs/Odometry` y RViz dibuja automáticamente la elipse alrededor del robot.

### 3.5 Wall-following con corrector P lateral

Ambos algoritmos comparten la pieza de bordeo. La idea: mantener una **distancia constante $d_{des}$ a la pared del lado elegido** con un controlador proporcional sobre el error lateral.

**Error y corrección:**

$$e_{lat} = d_{lateral} - d_{des}$$

$$\text{bias} = \mathrm{clip}\!\left(K_p \cdot e_{lat},\ -b_{max},\ +b_{max}\right)$$

donde $d_{lateral}$ es la distancia LiDAR al lado de la pared (`d_left` o `d_right` según `wall_side`), $d_{des} = 0.40$ m y $K_p = 1.0$. El clipping mantiene `bias` acotado para que el setpoint no quede tan torcido que el PID externo solo gire en sitio.

**Composición del setpoint:** en cada tick el bug calcula y publica un punto $(x_{sp}, y_{sp})$ en frame del mundo:

$$\begin{aligned}
\hat{\mathbf{f}} &= (\cos\theta,\ \sin\theta) \qquad \text{(vector frontal del robot)} \\
\hat{\mathbf{s}} &= (-\sin\theta,\ \cos\theta) \cdot \text{wall\_side} \qquad \text{(lateral hacia la pared)} \\
(x_{sp}, y_{sp}) &= (x, y) + \text{fwd} \cdot \hat{\mathbf{f}} + \text{bias} \cdot \hat{\mathbf{s}}
\end{aligned}$$

con $\text{fwd} = \min(\text{forward\_step},\ \text{dist\_goal})$. El recorte evita que el setpoint quede más lejos que el propio goal cuando el robot ya está cerca.

**Por qué un corrector P y no bang-bang:**

| Estrategia | Pro | Contra |
|---|---|---|
| Bang-bang | Trivial | Zigzag, $\Sigma_k$ crece rápido (cada $\omega$ grande mete ruido proporcional según el modelo del Reto 4) |
| **P-controller** | Suave, $\Sigma_k$ crece menos | Sintonía de $K_p$ |

### 3.6 Geometría del HIT y elección de lado

Cuando $d_{front} < \text{obstacle\_threshold}$ y el goal está más lejos que el obstáculo, se registra el **HIT**:

- $(x_{hit}, y_{hit}) \leftarrow (x, y)$ (pose actual)
- $d_{hit \to goal} \leftarrow \|p - g\|$ (distancia al goal en el HIT)
- $\text{wall\_side} \leftarrow$ resultado de `_choose_wall_side()`

**Política de elección de lado** (`_choose_wall_side`):

| Condición | wall_side | Significado |
|---|---|---|
| $\lvert d_L - d_R \rvert < 0.15$ m | $+1$ (default L) | Caso simétrico, consistente con bug_easy |
| $d_R > d_L$ | $+1$ (bordeo por sur) | Más espacio al sur → aprovecharlo |
| $d_L > d_R$ | $-1$ (bordeo por norte) | Más espacio al norte → aprovecharlo |

### 3.7 Fase de enganche (Phase 1 de WALL_FOLLOW)

En el HIT, **la pared está al FRENTE del robot, no al costado**. No se puede iniciar el wall-following P inmediatamente porque $d_{lateral} \approx \infty$.

Solución: una sub-fase de **enganche**:

1. Mientras `wall_engaged == False`, publicar un setpoint perpendicular al yaw, **hacia el lado opuesto a `wall_side`**. El controller PID con factor $\cos^2(\text{ang\_err})$ produce velocidad lineal $\approx 0$ → **el robot gira en sitio**.
2. Mientras gira, $d_{lateral}$ baja. Cuando $d_{lateral} < 1.5 \cdot d_{des}$ → `wall_engaged = True`, entra a la fase 2 (seguimiento P).

### 3.8 HIT secundario y watchdog topológico

**HIT secundario** (bug0): si durante WALL_FOLLOW aparece otro obstáculo y `dist_from_hit > 1.0` m (histeresis para no falsos positivos en el mismo obstáculo), opcionalmente flipea `wall_side`. Es lo que permite resolver `bug_medium` (2 obstáculos en cascada).

**Watchdog topológico** (bug2): si lleva > 4 m bordeando AND no se acercó al goal (progreso < 0.5 m desde el HIT), **flipea `wall_side`** y resetea el HIT. Esto saca al robot del callejón sin salida cuando entró al lado equivocado de un obstáculo cóncavo (caso `bug_hard`).

### 3.9 Salida de WALL_FOLLOW

**Bug 0** sale cuando se cumplen las dos condiciones:
- $d_{front} > \text{clear\_distance}$ (frente despejado)
- $d_{hit \to goal} - d_{actual \to goal} \geq \text{min\_hit\_progress}$ (progreso real)

**Bug 2** añade una tercera condición (la clave del algoritmo):
- $d_{m\text{-line}}(p) < \text{mline\_tolerance}$ (sobre la m-line)
- AND `left_mline == True` (ya nos alejamos al menos una vez)
- AND `wall_follow_dist > 1.0` m

Bug 2 además tiene un **override de proximidad** robusto: si $\text{dist\_goal} < 0.6$ m AND el goal está hacia adelante ($|\text{ang\_rel}| < 60°$) AND $d_{front} > \text{dist\_goal} + 0.1$ m (camino directo libre), salir. Los tres son necesarios — sin la verificación de $d_{front}$ el robot sale cerca del goal pero con una pared en medio y choca.

### 3.10 Controller PID (Reto 2)

El controller recibe un setpoint $(x_{sp}, y_{sp})$ y la odometría. Calcula:

$$\text{ang\_err} = \mathrm{atan2}(\sin(\phi - \theta), \cos(\phi - \theta)), \quad \phi = \mathrm{atan2}(\Delta y, \Delta x)$$

Comanda velocidades:

$$\omega = K_{p,w} \cdot \text{ang\_err} + K_{i,w} \int e_{ang}\, dt + K_{d,w} \cdot \dot{e}_{ang}$$

$$v = K_v \cdot \|\Delta\| \cdot \cos^2(\text{ang\_err})$$

El factor $\cos^2$ es importante: si el robot está desorientado (ej. `ang_err > 60°`), $v$ se reduce drásticamente para girar primero y avanzar después, **sin caer en el modo binario "v=0 o v máx"**. Esto permite que setpoints reactivos del bug se sigan suavemente.

Saturaciones: $v \in [0,\ 0.3]$ m/s, $\omega \in [-1.5,\ 1.5]$ rad/s.

---

## 4. Arquitectura del sistema (topics y dataflow)

```
                          ┌──────────────────┐
                          │  goal_publisher  │ ──/goal──→ ┐
                          └──────────────────┘            │
                                                          ▼
   ┌──────────────────┐  /scan    ┌──────────────────┐  ┌────────────┐
   │  Gazebo (LiDAR)  │ ────────► │ lidar_processor  │─►│  Bug 0/2   │
   └──────────────────┘           └──────────────────┘  └─────┬──────┘
        ▲     │ /joint_states                              /set_point
        │     ▼                                                │
        │   ┌────────────────────┐  /wr,/wl  ┌──────────────┐  ▼
        │   │ wheel_state_bridge │─────────► │ localisation │
        │   └────────────────────┘            │  + Σₖ 3x3   │
        │                                     └──────┬───────┘
        │                                            /odom
        │                                                ▼
        │                                  ┌─────────────────────────┐
        │                                  │ coordinate_transform    │ ─/tf→ RViz
        │                                  │ (odom → base_footprint) │
        │                                  └─────────────────────────┘
        │                                                │
        │                                                ▼
        │                                       ┌──────────────┐
   /cmd_vel ◄────────────────────────────────── │ controller   │
                                                │   (PID)      │
                                                └──────────────┘
```

---

## 5. Catálogo de nodos

| Nodo | Archivo | Rol | Frecuencia |
|------|---------|-----|------------|
| `gazebo` | (Gazebo Harmonic) | Simulador físico: ruedas, LiDAR, sensor de joint_states | 1 kHz física, ~50 Hz sensores |
| `parameter_bridge` | (ros_gz_bridge) | Convierte mensajes Gazebo ↔ ROS 2 | event-driven |
| `robot_state_publisher` | (ROS) | Publica TF estática del URDF (base_link → wheels, lidar, etc.) | event-driven |
| `lidar_processor` | `perception/lidar_processor.py` | Sectoriza `/scan` en `d_front`, `d_left`, `d_right` | tasa de `/scan` (~10 Hz) |
| `wheel_state_bridge` | `perception/wheel_state_bridge.py` | Convierte `/joint_states` → `/wr`, `/wl` | tasa de `/joint_states` (~50 Hz) |
| `localisation` | `localisation/localisation.py` | Dead reckoning + propagación $\Sigma_k$ | **50 Hz** (timer fijo, `dt=0.02 s`) |
| `coordinate_transform` | `localisation/coordinate_transform.py` | Convierte `/odom` en TF dinámico `odom → base_footprint` | tasa de `/odom` (50 Hz) |
| `goal_publisher` | `control/goal_publisher.py` | Publica el goal final $(x_T, y_T)$ y un marker verde para RViz | 1 Hz |
| `bug0` o `bug2` | `navigation/bug{0,2}.py` | FSM Bug: decide setpoint dinámico | **10 Hz** (`control_period=0.1 s`) |
| `controller` | `control/controller.py` | PID de posición: `/set_point` + `/odom` → `/cmd_vel` | 50 Hz (`dt=0.02 s`) |
| `static_transform_publisher` | (ROS) | Publica TF estática `map → odom` (identidad) | event-driven |
| `rviz2` | (ROS) | Visualización 3D | render-driven |

---

## 6. Catálogo de topics y mensajes

### 6.1 Topics de simulación (Gazebo ↔ ROS)

| Topic | Tipo | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | Todos (use_sim_time=True) |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (LiDAR plugin) | `lidar_processor`, RViz |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo (joint_state plugin) | `wheel_state_bridge`, `robot_state_publisher` |
| `/cmd_vel` | `geometry_msgs/Twist` | `controller` | Gazebo (DiffDrive plugin) |
| `/ground_truth` | `nav_msgs/Odometry` | Gazebo (odometry plugin) | (debug, opcional) |

### 6.2 Topics internos del Bug

| Topic | Tipo | Publisher | Subscribers | Significado |
|-------|------|-----------|-------------|-------------|
| `/wr` | `std_msgs/Float32` | `wheel_state_bridge` | `localisation` | Velocidad rueda derecha [rad/s] |
| `/wl` | `std_msgs/Float32` | `wheel_state_bridge` | `localisation` | Velocidad rueda izquierda [rad/s] |
| `/odom` | `nav_msgs/Odometry` | `localisation` | `coordinate_transform`, `bug0`/`bug2`, `controller`, RViz | Pose estimada + $\Sigma_k$ |
| `/tf` | `tf2_msgs/TFMessage` | `coordinate_transform`, `robot_state_publisher`, `static_transform_publisher` | RViz, todos los nodos TF-aware | Árbol de transformaciones |
| `/bug/d_front` | `std_msgs/Float32` | `lidar_processor` | `bug0`/`bug2` | Distancia mínima en cono frontal ±30° [m] |
| `/bug/d_left` | `std_msgs/Float32` | `lidar_processor` | `bug0`/`bug2` | Distancia mínima en cono izquierdo (+90° ±45°) [m] |
| `/bug/d_right` | `std_msgs/Float32` | `lidar_processor` | `bug0`/`bug2` | Distancia mínima en cono derecho (-90° ±45°) [m] |
| `/goal` | `geometry_msgs/Point` | `goal_publisher` | `bug0`/`bug2` | Goal final $(x_T, y_T)$ |
| `/set_point` | `geometry_msgs/Point` | `bug0`/`bug2` | `controller` | **Waypoint dinámico** que el PID debe perseguir ahora mismo |
| `/bug/state` | `std_msgs/String` | `bug0`/`bug2` | (debug / RViz) | `"GO_TO_GOAL"`, `"WALL_FOLLOW"`, `"GOAL_REACHED"` |
| `/bug/markers` | `visualization_msgs/MarkerArray` | `bug0`/`bug2` | RViz | Goal, m-line (bug2), radio de obstáculo |
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | `goal_publisher` | RViz | Esfera verde en el goal |
| `/next_point` | `std_msgs/Bool` | `controller` | (debug) | `True` una sola vez al alcanzar el setpoint |

### 6.3 Diferencia clave: `/goal` vs `/set_point`

| Topic | Quién publica | Cada cuánto | Qué representa |
|-------|---------------|-------------|----------------|
| `/goal` | `goal_publisher` | 1 Hz, **siempre el mismo** | El **goal final** que el robot tiene que alcanzar |
| `/set_point` | `bug0`/`bug2` | 10 Hz, **cambia constantemente** | El **siguiente waypoint** que el PID debe perseguir. Cuando el bug está en GO_TO_GOAL es igual al goal; cuando está en WALL_FOLLOW es un punto lateral al frente del robot |

Esta separación es lo que **cumple el requisito del PDF de reutilizar el controller del Mini Challenge 2**: el bug solo decide *a dónde ir ahora*, el PID se encarga de *cómo llegar*.

---

## 7. Máquinas de estado implementadas

### 7.1 Bug 0 (`bug0.py`)

```
        ┌─────────────────────┐
        │     GO_TO_GOAL      │◄──────┐
        └──────────┬──────────┘       │
                   │                  │
       d_front < obs_th                │
       AND d_front < dist_goal-0.30   │ libre + progreso ≥ 0.40m
                   │                  │
                   ▼                  │
        ┌─────────────────────┐       │
        │    WALL_FOLLOW      │───────┘
        │  · enganche (gira)  │
        │  · wall-follow P    │
        │  · HIT secundario   │
        └──────────┬──────────┘
                   │
       dist_goal < goal_tol
                   │
                   ▼
        ┌─────────────────────┐
        │   GOAL_REACHED      │  (terminal)
        └─────────────────────┘
```

### 7.2 Bug 2 (`bug2.py`)

Idéntica máquina de estados pero la transición `WALL_FOLLOW → GO_TO_GOAL` exige:
- $d_{front} > \text{obs\_th}$
- $d_{m\text{-line}} < 0.35$ m
- Progreso desde HIT $> 0.10$ m
- `left_mline` (ya nos alejamos al menos una vez)
- `wall_follow_dist > 1.0` m

Más dos overrides de seguridad:
- **Proximidad al goal** (`dist_goal < 0.6` m + visibilidad confirmada)
- **Watchdog topológico** (4 m sin progreso → flip de lado)

---

## 8. Explicación a fondo de cada código

### 8.1 `perception/lidar_processor.py`

**Rol:** desacopla el algoritmo del LiDAR convirtiendo 360 rayos en 3 escalares.

**Algoritmo:** para cada uno de los 3 sectores (FRONT, LEFT, RIGHT), calcula el índice central en el arreglo `ranges[]` del `LaserScan` y recorre $\pm\text{half\_width}/\text{angle\_increment}$ índices alrededor con wrap-around. Devuelve el mínimo rango válido (descarta inf, NaN y > `max_valid_range`). Si no hay rayo válido devuelve `max_valid_range` ("no veo nada en ese sector").

**Sectores:**

```
        +90° (LEFT, ±45°)
              ↑
   FRONT  ──┼──→  0°  (frente, ±30°)
              ↓
        −90° (RIGHT, ±45°)
```

**Por qué solo 3 sectores:** Bug 0/2 sólo necesitan saber "¿hay algo enfrente?", "¿cuánto espacio a la izquierda?" y "¿cuánto a la derecha?". Tres `Float32` a 10 Hz son suficientes y mucho más fáciles de razonar que un array de 360 floats.

### 8.2 `perception/wheel_state_bridge.py`

**Por qué existe:** el plugin `gz-sim-joint-state-publisher-system` de Gazebo emite **un solo mensaje** `/joint_states` con velocidades de todas las juntas. El nodo `localisation` (heredado del Reto 4) espera **dos topics separados** `/wr` y `/wl`. Este bridge hace la conversión.

**Implementación:** parsea el campo `name[]` del `JointState`, identifica los joints `wheel_right_joint` y `wheel_left_joint` (configurable por parámetro), y publica cada `velocity[i]` correspondiente como `Float32` en `/wr` o `/wl`.

### 8.3 `localisation/localisation.py` — Reutilizado del Reto 4

**Rol:** estimación de pose por dead-reckoning + propagación de covarianza.

**Pipeline (50 Hz):**

1. Lee últimas $(\omega_r, \omega_l)$ recibidas.
2. Convierte a $(v, \omega)$ con el modelo cinemático del Puzzlebot.
3. Integra Euler para actualizar la pose $(x, y, \theta)$.
4. Propaga $\Sigma_k = H \Sigma_{k-1} H^\top + Q$ (ver §3.4).
5. Construye un `nav_msgs/Odometry` con la pose en `pose.pose` y la matriz $\Sigma_k$ mapeada al arreglo `pose.covariance[36]` (los 9 cruces relevantes para $(x, y, \theta)$, el resto en cero).
6. Publica en `/odom`.

**Detalles clave:**
- $H$ y $\nabla\omega$ se evalúan en $\theta_{k-1}$ (el yaw **antes** de integrar), no en el yaw nuevo. Por eso el código guarda `th_prev = self.th` antes de la integración.
- $\Sigma_k$ inicia en cero (al arrancar sabemos exactamente dónde estamos). En dead-reckoning sólo crece — no hay observaciones que la corrijan, eso sería un Kalman con LiDAR/cámara.
- $\Sigma_{\Delta,k}$ depende linealmente de $|\omega|$ de cada rueda: si la rueda no se mueve, no contribuye al ruido.

### 8.4 `localisation/coordinate_transform.py`

**Rol:** publicar la transformación TF dinámica `odom → base_footprint` para que RViz coloque el robot y la elipse de covarianza en el lugar correcto.

**Implementación:** suscribe a `/odom`, en cada callback construye un `geometry_msgs/TransformStamped` copiando posición y orientación, y lo difunde por `TransformBroadcaster`. Usa el `stamp` del mensaje `/odom` (no `now()`) para que TF y covarianza estén sincronizados.

### 8.5 `control/goal_publisher.py`

**Rol:** publicar el goal final $(x_T, y_T)$ y un marcador visual para RViz.

**Implementación:** timer a 1 Hz, publica:
- `geometry_msgs/Point` en `/goal` (lo lee el bug)
- `visualization_msgs/MarkerArray` en `/visualization_marker_array` con una esfera verde en el goal (RViz)

El target se configura por parámetro `target_x`, `target_y` (configurable desde el launch).

### 8.6 `control/controller.py` — Reutilizado del Reto 2 ⭐

**Rol:** PID de posición que recibe `/set_point` y produce `/cmd_vel`.

**Pipeline (50 Hz):**

1. Lee setpoint actual (`target_x`, `target_y`) y pose actual.
2. Calcula `dist` y `ang_err`.
3. Si `dist > dist_tolerance`:
   - $\omega = K_p \cdot \text{ang\_err} + K_i \int + K_d \cdot \dot{e}$
   - $v = K_v \cdot \text{dist} \cdot \cos^2(\text{ang\_err})$
   - Satura: $v \in [0, 0.3]$, $\omega \in [-1.5, 1.5]$.
4. Publica `Twist` en `/cmd_vel` y `Bool(True)` una vez en `/next_point` al alcanzar.

**Detalle no obvio:** el callback de `/set_point` **detecta** si el nuevo target está muy lejos (`> 0.5 m` del anterior) o si es un refinamiento. Solo resetea el integrador y derivativo si es un "jump grande" — sin esto, el bug que reenvía setpoints cada 100 ms causaría picos en el derivativo cada tick.

### 8.7 `navigation/bug0.py` — Núcleo del Task 1 ⭐

**Rol:** FSM Bug 0 minimalista. 3 estados, 8 parámetros físicos.

**Suscripciones:** `/odom`, `/goal`, `/bug/d_front`, `/bug/d_left`, `/bug/d_right`.
**Publica:** `/set_point`, `/bug/state`, `/bug/markers`.

**Pipeline de cada tick (10 Hz):**

1. Calcular `dist_goal` desde `/odom`.
2. Si `dist_goal < goal_tolerance` → `GOAL_REACHED`, terminar.
3. Si en **GO_TO_GOAL**:
   - Si `d_front > dist_goal - 0.30` (lo de enfrente está más lejos que el goal) → setpoint = goal.
   - Si `d_front < obstacle_threshold` → registrar HIT, calcular `wall_side`, entrar a `WALL_FOLLOW`.
4. Si en **WALL_FOLLOW**:
   - **Salida**: si `wall_engaged` AND `d_front > clear_distance` AND `progreso ≥ min_hit_progress` → volver a `GO_TO_GOAL`.
   - **Enganche**: si `d_lateral < 1.5·wall_distance` → `wall_engaged = True`.
   - Si no engaged → publicar setpoint perpendicular al yaw (gira en sitio, fase 1).
   - **HIT secundario**: si `d_front < obs_th` AND `dist_from_hit > 1.0` AND lateral opuesto tiene > 30 cm más espacio → flip lado.
   - Si nada de lo anterior, publicar setpoint con corrector P lateral (fase 2).

### 8.8 `navigation/bug2.py` — Núcleo del Task 2 ⭐

Idéntico a Bug 0, con cinco diferencias:

1. **Estado adicional**: $(x_s, y_s)$ se fija con la primera lectura de odom. Define la m-line. Si llega un goal nuevo por `/goal`, se reinicia desde la pose actual.
2. **Función `_distance_to_mline(x, y)`** implementa la fórmula clásica (§3.2.1).
3. **Condición de salida diferente** (§3.9) basada en la m-line, NO solo en frente libre.
4. **Override de proximidad estricto** con verificación de `d_front >= dist_goal + 0.1`.
5. **Watchdog topológico** (§3.8) que flipea `wall_side` cuando lleva 4 m sin progresar.

Visualiza la m-line en RViz como línea cyan.

---

## 9. Launch files

### 9.1 `bringup_launch.py` — Capa de simulación

Levanta lo común a Task 1 y Task 2:

- **Gazebo Harmonic** con el `.world` elegido (argument `world:=...`).
- **`robot_state_publisher`** con el URDF del Puzzlebot generado por `xacro`.
- **`ros_gz_sim.create`** que spawnea el robot en la pose `(x, y, yaw)`.
- **`parameter_bridge`** de `ros_gz_bridge` con los siguientes bridges:
  - `/clock` (gz → ros)
  - `/scan` (gz → ros)
  - `/cmd_vel` (ros → gz)
  - `/ground_truth` (gz → ros)
  - `/joint_states` (gz → ros)
- **Env vars de OpenGL** (`OGRE_RTT_MODE=Copy`, `__GLX_VENDOR_LIBRARY_NAME=mesa`, `QT_QPA_PLATFORM=xcb`) para evitar el crash de NVIDIA en el sistema actual (driver roto).

Los nodos posteriores se lanzan con un `TimerAction(period=3.0)` para dar tiempo a que Gazebo emita `/clock`.

### 9.2 `bug0_launch.py` — Task 1

Incluye `bringup_launch.py` y añade:

| Nodo | Parámetros pasados |
|------|---------------------|
| `static_transform_publisher` | `map → odom` identidad |
| `lidar_processor` | (defaults) |
| `wheel_state_bridge` | (defaults) |
| `localisation` | `x_init`, `y_init`, `theta_init` desde args |
| `coordinate_transform` | (defaults) |
| `goal_publisher` | `target_x`, `target_y` desde args |
| `controller` | `dist_tolerance=0.10` |
| `bug0` | `target_x`, `target_y` desde args |
| `rviz2` | `-d puzzlebot_bug.rviz` |

Los nodos de navegación se lanzan con `TimerAction(period=5.0)` después del bringup, para que el bridge haya publicado `/clock` y los nodos con `use_sim_time=True` lo tengan disponible.

### 9.3 `bug2_launch.py` — Task 2

Idéntico al de Bug 0 pero ejecuta el nodo `bug2` en lugar de `bug0`.

---

## 10. Estructura del paquete

```
Week6/Challenge/
├── README.md                         ← este documento
├── run.sh                            ← launcher interactivo
├── output/                           ← evidencias (TF PDFs, grabaciones)
├── MCR2_Mini_Challenge.pdf           ← enunciado original
└── puzzlebot_bug/                    ← paquete ROS 2 (puzzlebot_bug_w6)
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/puzzlebot_bug_w6
    ├── worlds/                       ← 6 escenarios propios
    │   ├── bug_easy.world            ─ 1 obstáculo simple
    │   ├── bug_medium.world          ─ 2 obstáculos en cascada
    │   ├── bug_hard.world            ─ U-trap (caso límite Bug 0)
    │   ├── bug_office.world          ─ cuarto cerrado + tabique central
    │   ├── bug_arena.world           ─ ladrillos dispersos + bolsillo
    │   └── bug_hard_plus.world       ─ U + ladrillos en approach
    ├── rviz/
    │   └── puzzlebot_bug.rviz        ─ configuración con TF, scan, marcadores
    ├── launch/
    │   ├── bringup_launch.py         ─ Gazebo + robot + bridges + env GL
    │   ├── bug0_launch.py            ─ Task 1
    │   └── bug2_launch.py            ─ Task 2
    └── puzzlebot_bug/                ← código Python por dominio
        ├── perception/
        │   ├── lidar_processor.py    ─ /scan → distancias sectorizadas
        │   └── wheel_state_bridge.py ─ joint_states → /wr, /wl
        ├── navigation/   ⭐
        │   ├── bug0.py               ─ FSM Bug 0
        │   └── bug2.py               ─ FSM Bug 2 + m-line
        ├── localisation/
        │   ├── localisation.py       ─ reutilizado del Reto 4 (Σₖ)
        │   └── coordinate_transform.py
        └── control/
            ├── controller.py         ─ PID del Reto 2 (cmd_vel)
            └── goal_publisher.py     ─ publica (x_T, y_T) en /goal
```

---

## 11. Cómo correrlo

### 11.1 Pre-requisitos

| Componente | Cómo se instala |
|---|---|
| ROS 2 Humble | (ya instalado) |
| Gazebo Harmonic (Sim 8) | `sudo apt install gz-harmonic` |
| Bridge ROS↔Gazebo | `sudo apt install ros-humble-ros-gzharmonic` |
| Paquetes MCR2 | Ya están en `Week5/Gazebo Simulator/` |

### 11.2 Compilar

Una sola vez. Construimos nuestro paquete + los dos del profesor:

```bash
cd "/home/alfonso/Documents/8 Semestre/manchester_bloque"
colcon build \
    --packages-select puzzlebot_bug_w6 puzzlebot_gazebo puzzlebot_description \
    --paths \
        "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week6/Challenge/puzzlebot_bug" \
        "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week5/Gazebo Simulator/puzzlebot_gazebo" \
        "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week5/Gazebo Simulator/puzzlebot_description"
source install/setup.bash
```

(El `run.sh` hace todo esto en su menú "Recompilar el paquete".)

### 11.3 Lanzar con el script interactivo

```bash
bash "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week6/Challenge/run.sh"
```

Menús:

- **Task 1 → Bug 0**: 8 opciones (3 canónicos + 3 extendidos + default profesor + personalizado).
- **Task 2 → Bug 2**: idem.
- **Diagnóstico ROS**: nodos, topics, `/bug/state` en vivo, distancias del LiDAR, TF tree (PDF), rqt_graph.

### 11.4 Comandos crudos (alternativa)

```bash
# Bug 0 escenario fácil
ros2 launch puzzlebot_bug_w6 bug0_launch.py

# Bug 2 escenario difícil (forma de U)
ros2 launch puzzlebot_bug_w6 bug2_launch.py world:=bug_hard.world \
    x:=-2.5 target_x:=2.5

# Personalizar todo
ros2 launch puzzlebot_bug_w6 bug0_launch.py \
    world:=bug_medium.world target_x:=4.0 target_y:=0.0 x:=0.0 y:=0.0
```

### 11.5 Inspeccionar en vivo

**Terminal 1** — sistema completo:
```bash
./run.sh  →  1 (Task 1)  →  1 (bug_easy)
```

**Terminal 2** — observar el estado del bug en tiempo real:
```bash
ros2 topic echo /bug/state
# imprime: GO_TO_GOAL → WALL_FOLLOW → GO_TO_GOAL → GOAL_REACHED
```

**Topics útiles para debug:**
```bash
ros2 topic echo /set_point     # waypoint que el bug le da al PID
ros2 topic echo /bug/d_front   # distancia LiDAR frontal
ros2 topic echo /odom --field pose.pose.position
ros2 topic hz /cmd_vel         # frecuencia con que el robot recibe comandos
```

---

## 12. Mundos disponibles

| World | Dificultad | Start | Goal | Bug 0 | Bug 2 |
|---|---|---|---|:---:|:---:|
| `bug_easy.world` | Fácil | (0, 0) | (3.0, 0.0) | ✓ | ✓ |
| `bug_medium.world` | Media | (0, 0) | (4.0, 0.0) | ✓ | ✓ |
| `bug_hard.world` | Difícil (U-trap) | (-2.5, 0) | (2.5, 0.0) | ✗ (limitación del algoritmo) | ✓ |
| `bug_office.world` | Cuarto cerrado | (-1.3, -1.5) | (1.3, 1.5) | depende | depende |
| `bug_arena.world` | Ladrillos dispersos | (0, 0) | (2.4, 2.0) | depende | depende |
| `bug_hard_plus.world` | U + ladrillos | (-2.5, 0) | (2.5, 0.0) | ✗ | depende |

> `bug_hard` con Bug 0 falla **por diseño del algoritmo**, no por bug del código. Bug 0 es greedy y greedy no funciona en obstáculos cóncavos. **Bug 2 lo resuelve** gracias a la m-line + watchdog topológico.

---

## 13. Tabla de parámetros

### 13.1 Bug 0 / Bug 2 (compartidos)

| Parámetro | Default | Unidad | Significado |
|---|---|---|---|
| `control_period` | 0.10 | s | $\Delta t$ del lazo (10 Hz) |
| `goal_tolerance` | 0.25 | m | Radio para considerar GOAL_REACHED |
| `obstacle_threshold` | 0.50 | m | $d_{front}$ que activa WALL_FOLLOW |
| `clear_distance` | 0.90 | m | $d_{front}$ libre para salir de WALL_FOLLOW |
| `wall_distance` | 0.40 | m | Distancia lateral deseada a la pared |
| `kp_wall` | 1.00 | — | Ganancia del corrector P lateral |
| `forward_step` | 0.80 | m | Look-ahead del setpoint |
| `min_hit_progress` | 0.40 | m | Progreso mínimo desde HIT para salir |

### 13.2 Bug 2 (extras)

| Parámetro | Default | Unidad | Significado |
|---|---|---|---|
| `mline_tolerance` | 0.35 | m | $\varepsilon$ para considerar "sobre la m-line" |
| Watchdog topológico | 4.0 | m | Distancia bordeada sin progreso → flip lado |
| Override de proximidad | 0.6 | m | Distancia al goal para forzar salida (con check de visibilidad) |

### 13.3 Controller PID (`control/controller.py`)

| Parámetro | Default | Significado |
|---|---|---|
| `k_v` | 0.5 | Ganancia P lineal |
| `kp_w`, `ki_w`, `kd_w` | 1.0, 0.01, 0.1 | PID angular |
| `dist_tolerance` | 0.05 m | Radio de aceptación de waypoint |

### 13.4 Localisation (`localisation/localisation.py`)

| Parámetro | Default | Significado |
|---|---|---|
| `wheel_radius` | 0.05 m | Radio físico de la rueda |
| `wheelbase` | 0.19 m | Distancia entre ruedas |
| `sample_time` | 0.02 s | $\Delta t$ del integrador (50 Hz) |
| `k_r`, `k_l` | 0.05, 0.05 | Ganancias de ruido por rueda |

### 13.5 LiDAR processor (`perception/lidar_processor.py`)

| Parámetro | Default | Significado |
|---|---|---|
| `front_half_width_deg` | 30° | Half-anchura del cono frontal |
| `side_half_width_deg` | 45° | Half-anchura de los conos laterales |
| `max_valid_range` | 10.0 m | Cota superior; rayos > esto se descartan |

---

## 14. Cumplimiento del enunciado

- [x] **Bug 0** (Task 1) con FSM explícita (`GO_TO_GOAL`, `WALL_FOLLOW`, `GOAL_REACHED`).
- [x] **Bug 2** (Task 2) con tracking de m-line + watchdog topológico.
- [x] **Reutiliza el controller del Mini Challenge 2** (la pipeline real es bug → `/set_point` → controller → `/cmd_vel`).
- [x] **Considera la covarianza $\Sigma_k$ creciente** (`localisation.py` es el mismo nodo del Reto 4; RViz dibuja la elipse).
- [x] Probado en **6 mundos propios** + mundos oficiales del profesor.
- [x] Sampling time fijo: 10 Hz Bug, 50 Hz controller, 50 Hz localisation.
- [x] **Sólo NumPy + stdlib** (sin librerías externas).
- [x] Sin teleop ni intervención humana.
- [x] Launches dedicados para cada algoritmo.
- [x] **Bug 2 cierra el caso `bug_hard`** que Bug 0 no puede resolver (demostración pedagógica del valor de la m-line).

---

## 15. Mejoras justificadas

> *"Improvements to the algorithms are encouraged..."* — PDF del reto.

### 15.1 Wall-following con corrector P en lugar de bang-bang
Trayectorias suaves, $\Sigma_k$ crece menos. Ver §3.5.

### 15.2 Setpoint $(x, y)$ al PID externo, no `/cmd_vel` directo
Permite reutilizar literalmente el controller del Mini Challenge 2 (cumple el requisito del PDF) y separa "qué punto perseguir" de "cómo llegar a él".

### 15.3 Sectorización del LiDAR (`lidar_processor.py`)
Desacopla el sensor del algoritmo. El bug consume sólo 3 escalares, lo que hace la lógica testeable y portable.

### 15.4 Fase de enganche en WALL_FOLLOW
En el HIT la pared está al frente, no al costado. Sin la fase de enganche, el corrector P inicial es errático porque $d_{lateral} \approx \infty$. La fase 1 gira en sitio hasta que $d_{lateral}$ es realista, después arranca el wall-follow normal.

### 15.5 HIT secundario para multi-obstáculo
Cuando aparece otro obstáculo mientras bordeas el primero, flipea `wall_side`. Con histeresis (`dist_from_hit > 1.0` m) para evitar falsos positivos en el mismo obstáculo. Es lo que resuelve `bug_medium`.

### 15.6 Watchdog topológico (Bug 2)
Si lleva 4 m bordeando sin acercarse al goal, flipea `wall_side`. Saca al robot del callejón sin salida cuando entró al lado equivocado de la U en `bug_hard`.

### 15.7 Override de proximidad estricto (Bug 2)
Salir cerca del goal SÓLO si el camino directo está libre (`d_front > dist_goal + 0.1`). Sin esta verificación, el robot salía cerca del goal pero con una pared en medio y chocaba.

### 15.8 Setpoint con `forward_step` recortado por `dist_goal`
`fwd = min(forward_step, dist_goal)` evita que el setpoint quede más lejos que el propio goal cuando el robot ya está cerca — sin esto el robot orbita el goal sin entrar.

### 15.9 Detección de "jump grande" en el controller
El callback de `/set_point` solo resetea el integrador/derivativo del PID si el nuevo target está > 0.5 m del anterior. Sin esto, los refinamientos de 10 Hz del bug producirían picos derivativos cada tick.

### 15.10 Workaround de rendering OpenGL (en `bringup_launch.py`)
En equipos con driver NVIDIA en mal estado (`nvidia-smi` falla), forzar `__GLX_VENDOR_LIBRARY_NAME=mesa` y `OGRE_RTT_MODE=Copy` permite que Gazebo Harmonic + RViz arranquen usando Intel GPU + Mesa.

---

## 16. Troubleshooting

### "Gazebo no abre / cuelga"
Asegúrate de tener Harmonic, no Classic:
```bash
gz sim --version    # debe imprimir "Gazebo Sim, version 8.x"
```

### Rendering: pantalla negra en Gazebo o `QGLXContext: Failed to create dummy context`
El driver NVIDIA puede estar roto. Verifica:
```bash
nvidia-smi -L
# Si dice "No devices were found" o similar, NVIDIA está caído.
```
El `run.sh` y `bringup_launch.py` fuerzan Mesa/Intel por default. Si NVIDIA funciona y quieres usarla, modifica las env vars en `run.sh` (función `launch_bug()`).

### El robot no aparece o no recibe `/scan`
```bash
ros2 topic list | grep -E "scan|cmd_vel|joint_states|odom"
```
Debes ver `/scan`, `/cmd_vel`, `/joint_states`, `/odom`.

### "Package 'puzzlebot_bug_w6' not found"
No sourceaste el `install/setup.bash` después de compilar:
```bash
source "/home/alfonso/Documents/8 Semestre/manchester_bloque/install/setup.bash"
```

### El robot gira en sitio sin avanzar
El controller PID tiene factor $\cos^2(\text{ang\_err})$ que reduce $v$ cuando el robot está desorientado. Es esperado en la fase de enganche (WALL_FOLLOW fase 1). Si pasa de forma persistente en GO_TO_GOAL, revisa que `/odom` se esté publicando correctamente y que el bug esté emitiendo `/set_point` distinto al de hace 5 segundos:
```bash
ros2 topic echo /set_point
ros2 topic echo /odom --field pose.pose.position
```

### Bug 0 nunca llega en bug_hard
Es esperado, **es la limitación matemática de Bug 0** en obstáculos cóncavos. Usa Bug 2 (Task 2) para ese escenario.

### Bug 2 entra al callejón sin salida de bug_hard
El watchdog topológico debe disparar tras 4 m de bordeo sin progreso. Si tarda mucho, baja la constante en `_do_wall_follow` de `bug2.py` (`self.wall_follow_dist > 4.0`).

### "ros_gz_bridge: Creating ROS->GZ Bridge: [/cmd_vel...]" pero el robot no se mueve
Verifica que el `parameter_bridge` esté traduciendo en la dirección correcta y que el plugin DiffDrive de Gazebo esté en el URDF:
```bash
ros2 topic info /cmd_vel
# Debe tener 1 publisher (controller) y 1 subscriber (bridge)
ros2 topic hz /cmd_vel
# Debe latir a ~50 Hz
```

---

## Créditos

- **Algoritmos Bug**: Choset et al., *Principles of Robot Motion* (2005); Lumelsky & Stepanov (1986).
- **Plataforma**: Puzzlebot Jetson Lidar Edition (Manchester Robotics).
- **Stack**: ROS 2 Humble + Gazebo Harmonic + RViz 2.

> Manchester Robotics — *{Learn, Create, Innovate};*
