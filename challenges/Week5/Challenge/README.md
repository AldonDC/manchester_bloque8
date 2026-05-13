# Mini Challenge 4 — Propagación de Incertidumbre en Odometría
### Week 5 · Manchester Robotics × NVIDIA

> Autor: **Alfonso Diaz** · Curso: Robótica móvil · Plataforma: **Puzzlebot Jetson Lidar Edition**
> ROS 2 · Python · NumPy (puro, sin librerías externas)

---

## 1. ¿Qué pide el reto?

El reto consiste en **modelar y visualizar la incertidumbre** que acumula el robot conforme se desplaza. Cuando un robot diferencial estima su posición sólo con los encoders (dead reckoning), nunca sabe con exactitud dónde está: cada rueda introduce ruido y el error se propaga. El objetivo es **cuantificar ese error** con una matriz de covarianza Σ y dibujarlo en RVIZ como una **elipse de confianza** alrededor del robot.

### Tareas del PDF

| Task | Descripción |
|------|-------------|
| **Task 1** | Completar la matriz 3x3 de covarianza dentro del mensaje `nav_msgs/Odometry` y dibujar la elipse en RVIZ. |
| **Task 2** | Calibrar los parámetros `k_r` y `k_l` del modelo de ruido por rueda mediante experimentos repetibles (recta y rotación). |

### Reglas (del PDF)

- ✅ Solo se permite **Python estándar + NumPy**. Prohibido `scipy`, `transforms3d`, etc.
- ✅ EDOs resueltas **a mano** (Euler).
- ✅ Reutilizar el simulador y el nodo de localización del **Mini Challenge anterior** (Week 4).
- ✅ Sin teleop ni intervención humana durante la simulación.

---

## 2. Modelo matemático

### 2.1 Pose como variable aleatoria

La pose del robot en el paso `k` se modela como una **Gaussiana 3D**:

```
sₖ  ~  𝒩(μₖ, Σₖ)
```

donde `μₖ = [x, y, θ]ᵀ` es la pose más probable (dead reckoning) y `Σₖ` es la matriz 3×3 que describe la dispersión.

### 2.2 Propagación de la covarianza

```
Σₖ  =  Hₖ · Σₖ₋₁ · Hₖᵀ  +  Qₖ
```

| Símbolo | Significado | Tamaño |
|---------|-------------|--------|
| `Hₖ`  | Jacobiano del modelo cinemático respecto al estado previo | 3×3 |
| `Σₖ₋₁` | Covarianza acumulada hasta el paso anterior | 3×3 |
| `Qₖ`  | Error no-determinístico aportado por las ruedas en este paso | 3×3 |

#### Jacobiano del modelo `Hₖ`

```
        ⎡ 1   0   −Δt·v·sin(θₖ₋₁) ⎤
Hₖ  =   ⎢ 0   1    Δt·v·cos(θₖ₋₁) ⎥
        ⎣ 0   0           1        ⎦
```

#### Construcción de `Qₖ`

```
Qₖ  =  ∇ωₖ · Σ_Δ,k · ∇ωₖᵀ
```

Donde:

```
            r·Δt   ⎡ cos θ   cos θ  ⎤
∇ωₖ  =  ───── · ⎢ sin θ   sin θ  ⎥          (3×2)
             2     ⎣  2/L    −2/L  ⎦

           ⎡ k_r·|ω_r|       0      ⎤
Σ_Δ,k  =   ⎣      0      k_l·|ω_l| ⎦          (2×2)
```

`k_r` y `k_l` son los parámetros que el reto pide **calibrar**: representan cuánto ruido aporta cada rueda por unidad de velocidad angular.

### 2.3 Σ se "infla" siempre

La covarianza por dead reckoning **sólo puede crecer** (es semidefinida positiva y se le suma `Qₖ ≥ 0` en cada paso). La única forma de reducirla es agregar observaciones externas (LiDAR, IMU, GPS…), lo cual ya es **Kalman** y sale fuera de este reto.

---

## 3. Arquitectura del sistema

El diagrama queda **idéntico** al del PDF. Lo único nuevo es que dentro del nodo `Localisation` ahora se calcula y publica Σₖ.

```
┌────────────────────────┐                          ┌───────────┐
│  Trajectory Set Point  │ ─/set_point→ ┌──────────┐│ URDF/STL │
│      Generator         │              │Controller│└─────┬─────┘
└────────────────────────┘              └────┬─────┘      │
                                             │            ▼
                                       /cmd_vel    ┌───────────────────┐
                                             │     │   robot_state_pub │
                                             ▼     └──────┬────────────┘
                                       ┌────────────┐     │
                                       │ Real / Sim │     │
                                       │   Robot    │     │
                                       └────┬───────┘     │
                                            │ /wr /wl      │
                                            ▼              │
                                       ┌─────────────┐     │
                                       │Localisation │     │
                                       │ + Σₖ (3x3)  │ ──┐ │
                                       └──────┬──────┘   │ │
                                              │ /odom    │ │
                          ┌───────────────────┼──────────┘ │
                          ▼                   ▼            ▼
                  ┌──────────────┐  ┌────────────────┐  ┌──────┐
                  │coordinate_tf │  │joint_state_pub │  │ RVIZ │
                  │  (odom→base) │  │   (ruedas)     │  │ (Σ)  │
                  └──────┬───────┘  └────────┬───────┘  └──────┘
                         │ /tf              │ /joint_states
                         └──────────────────┘
```

---

## 4. Estructura del paquete

El código se organiza por **rol funcional** (no por orden cronológico), lo que hace que cada carpeta sea autoexplicativa y deja el corazón del reto (`localisation/`) bien aislado.

```
Week5/Challenge/
├── README.md                             ← este documento
├── run.sh                                ← launcher interactivo (menús)
├── output/                               ← PDFs del TF tree (generados)
├── MCR2_Mini_Challenge4.pdf              ← enunciado original
└── puzzlebot_sim/                        ← paquete ROS 2 (puzzlebot_sim_w5)
    ├── package.xml                       ← nombre ROS: puzzlebot_sim_w5
    ├── setup.py                          ← entry points de todos los nodos
    ├── setup.cfg
    ├── resource/puzzlebot_sim_w5
    ├── urdf/puzzlebot.urdf
    ├── meshes/*.stl
    ├── rviz/
    │   └── puzzlebot_covariance.rviz     ← display "Odometry > Covariance" activado
    ├── launch/
    │   ├── puzzlebot_challenge5_launch.py    ← Task 1 — sistema completo
    │   ├── experiment_straight_launch.py     ← Task 2 — recta 1 m
    │   └── experiment_rotation_launch.py     ← Task 2 — rotación en sitio
    └── puzzlebot_sim/                    ← código Python organizado por dominio
        │
        ├── simulation/                   ← genera la realidad virtual
        │   ├── kinematic_sim.py              Real/Sim Robot
        │   └── open_loop_driver.py           Inyector cmd_vel para Task 2
        │
        ├── localisation/   ⭐ NÚCLEO DEL RETO
        │   └── localisation.py               Dead reckoning + propagación Σₖ
        │
        ├── visualisation/                ← soporte de RViz
        │   ├── coordinate_transform.py       TF odom → base_footprint
        │   └── joint_state_publisher.py      Animación de ruedas
        │
        └── control/                      ← inteligencia autónoma
            ├── controller.py                 PID hacia waypoints
            └── trajectory_generator.py       Square / triangle / hexagon
```

**¿Por qué esta estructura?**
- `localisation/` queda en una carpeta dedicada → es el círculo rojo del diagrama del PDF.
- Cada subcarpeta agrupa nodos con una **única responsabilidad**.
- Los nombres son autoexplicativos: cualquier evaluador entiende el rol de cada parte sin leer código.

---

## 5. Explicación de los códigos

Esta sección documenta **qué hace cada archivo, cómo se publica/suscribe, y cómo está hecha la matemática por dentro**. El objetivo es que cualquier evaluador pueda entender el reto sin tener que leer el código línea por línea.

---

### 5.1  `localisation/localisation.py` — Núcleo del reto ⭐

**Rol:** estima la pose del robot por *dead reckoning* y propaga la matriz de covarianza Σₖ. Es el único nodo nuevo/modificado respecto a Week 4 y donde vive toda la matemática del Task 1.

**Contrato ROS:**

| Dirección | Topic | Tipo | Origen / Destino |
|-----------|-------|------|-------------------|
| Sub | `/wr` | `std_msgs/Float32` | viene del simulador |
| Sub | `/wl` | `std_msgs/Float32` | viene del simulador |
| Pub | `/odom` | `nav_msgs/Odometry` | RViz, Coordinate Transform, Joint State Pub |

Corre a frecuencia fija (50 Hz por defecto, `sample_time = 0.02 s`) controlado por un `Timer` de rclpy.

**Pipeline de cada ciclo (`_timer_callback`):**

1. **Cinemática inversa rueda → robot**
   $$v = \frac{r\,(\omega_r + \omega_l)}{2}, \qquad \omega = \frac{r\,(\omega_r - \omega_l)}{L}$$
2. **Integración de Euler de la pose** (las EDOs se resuelven a mano, como pide el PDF):
   $$x_k = x_{k-1} + v\cos(\theta_{k-1})\,\Delta t$$
   $$y_k = y_{k-1} + v\sin(\theta_{k-1})\,\Delta t$$
   $$\theta_k = \theta_{k-1} + \omega\,\Delta t$$
3. **Propagación de la covarianza** (`_propagate_covariance`).
4. **Publicación de `/odom`** con pose, twist y Σ embebida.

> Importante: los Jacobianos se evalúan con **θ del paso previo**. Por eso el código guarda `th_prev = self.th` ANTES de integrar.

#### Cómo está hecha la matemática de Σₖ

El modelo del PDF dice:

$$\boxed{\;\Sigma_k \;=\; H_k\,\Sigma_{k-1}\,H_k^{T}\;+\;Q_k\;}$$

donde cada pieza se construye así dentro de `_propagate_covariance`:

**(a) Jacobiano del modelo `H_k` (3×3).** Derivada parcial de la pose nueva respecto a la pose vieja. La columna de θ refleja cómo un error de orientación rota la posición:

$$H_k = \begin{bmatrix} 1 & 0 & -\Delta t \cdot v \cdot \sin(\theta_{k-1}) \\ 0 & 1 & \phantom{-}\Delta t \cdot v \cdot \cos(\theta_{k-1}) \\ 0 & 0 & 1 \end{bmatrix}$$

En código:
```python
H = np.array([
    [1.0, 0.0, -dt*v*s],
    [0.0, 1.0,  dt*v*c],
    [0.0, 0.0,      1.0],
])
```

**(b) Jacobiano respecto a las ruedas `∇ωₖ` (3×2).** Derivada parcial de la pose respecto a (ω_r, ω_l). El factor (r·Δt/2) sale de las definiciones de v y ω:

$$\nabla\omega_k = \frac{r\,\Delta t}{2}\begin{bmatrix} \cos\theta & \cos\theta \\ \sin\theta & \sin\theta \\ 2/L & -2/L \end{bmatrix}$$

**(c) Covarianza del ruido de entrada `Σ_Δ,k` (2×2).** Modelo del PDF: cada rueda aporta ruido proporcional a la magnitud de su velocidad. Si la rueda no gira, no aporta ruido:

$$\Sigma_{\Delta,k} = \begin{bmatrix} k_r\,|\omega_r| & 0 \\ 0 & k_l\,|\omega_l| \end{bmatrix}$$

**(d) `Q_k` — error inyectado este Δt.** Lleva la incertidumbre del espacio de ruedas al espacio de la pose:

$$Q_k = \nabla\omega_k \cdot \Sigma_{\Delta,k} \cdot \nabla\omega_k^{T}$$

```python
Q = grad_w @ Sigma_delta @ grad_w.T
```

**(e) Suma final.** El primer término propaga la incertidumbre vieja a través del modelo. El segundo agrega la nueva. Por construcción Σ es semidefinida positiva y **sólo puede crecer** porque no hay observaciones que la corrijan (no es Kalman).

```python
return H @ self.Sigma @ H.T + Q
```

#### Cómo se mete Σₖ (3×3) en `pose.covariance` (36 floats)

`nav_msgs/Odometry` espera la covarianza como una 6×6 aplanada en orden `(x, y, z, φ, ψ, θ)`. Como el Puzzlebot es 2D, sólo se rellenan las 9 entradas que combinan (x, y, θ):

| Σ entry | índice plano |   | Σ entry | índice plano |   | Σ entry | índice plano |
|---------|--------------|---|---------|--------------|---|---------|--------------|
| σ_xx | 0  |   | σ_xy | 1  |   | σ_xθ | 5  |
| σ_yx | 6  |   | σ_yy | 7  |   | σ_yθ | 11 |
| σ_θx | 30 |   | σ_θy | 31 |   | σ_θθ | 35 |

El resto queda en cero, incluida toda la covarianza de velocidad (como permite el enunciado del reto). Con esto RViz dibuja la elipse de posición + el cono de orientación automáticamente.

---

### 5.2  `simulation/kinematic_sim.py` — Real / Sim Robot

**Rol:** la "planta física virtual". Sustituye al Puzzlebot real durante el desarrollo. Recibe comandos de velocidad y publica las velocidades de rueda que ve la Localisation.

**Contrato ROS:**

| Dirección | Topic | Tipo |
|-----------|-------|------|
| Sub | `/cmd_vel` | `geometry_msgs/Twist` |
| Pub | `/wr` | `std_msgs/Float32` |
| Pub | `/wl` | `std_msgs/Float32` |
| Pub | `/pose_sim` | `geometry_msgs/PoseStamped` (ground truth) |

**Matemática:** modelo diferencial integrado por Euler.

- Integración de pose (idéntica a Localisation):
  $$x_k = x_{k-1} + v\cos(\theta_{k-1})\,\Delta t, \quad y_k = \ldots, \quad \theta_k = \ldots$$
- Cinemática inversa robot → rueda:
  $$\omega_r = \frac{v + \omega\,L/2}{r}, \qquad \omega_l = \frac{v - \omega\,L/2}{r}$$

La rueda externa gira más rápido al girar porque recorre una circunferencia mayor.

---

### 5.3  `simulation/open_loop_driver.py` — Helper para Task 2

**Rol:** nodo auxiliar que **inyecta un `cmd_vel` constante** durante un tiempo fijo. Se usa SOLO en los experimentos de calibración para que la entrada al sistema sea determinística (sin que el PID contamine la medición).

**Contrato ROS:**

| Dirección | Topic | Tipo |
|-----------|-------|------|
| Pub | `/cmd_vel` | `geometry_msgs/Twist` |

**Lógica:** durante `T` segundos publica `(v, ω)` constantes. Después de `T`, publica `(0, 0)` para detener el robot pero mantenerlo vivo en RViz y poder observar la elipse final.

Configuraciones:
- Recta de 1 m → `v = 0.15 m/s, ω = 0, T = 6.67 s` (≈ 1 m a 0.15 m/s).
- Rotación → `v = 0, ω = π/2 rad/s, T = 4 s` (≈ una vuelta completa).

---

### 5.4  `visualisation/coordinate_transform.py` — Difusor TF

**Rol:** convierte cada `/odom` en una transformación TF para que RViz sepa **dónde colocar la elipse** y el modelo 3D del robot.

**Contrato ROS:**

| Dirección | Topic | Tipo |
|-----------|-------|------|
| Sub | `/odom` | `nav_msgs/Odometry` |
| Broadcast | `/tf` | `geometry_msgs/TransformStamped` |

**Lógica:** copia la pose del mensaje a un `TransformStamped` con:
- `header.frame_id = "odom"` (padre)
- `child_frame_id = "base_footprint"` (hijo)

Usa el mismo `timestamp` del `/odom` para que RViz sincronice la TF con el mensaje de covarianza del mismo ciclo.

---

### 5.5  `visualisation/joint_state_publisher.py` — Animación de ruedas

**Rol:** publica el ángulo acumulado de cada rueda para que el `robot_state_publisher` haga girar las ruedas del URDF en RViz.

**Contrato ROS:**

| Dirección | Topic | Tipo |
|-----------|-------|------|
| Sub | `/odom` | `nav_msgs/Odometry` |
| Pub | `/joint_states` | `sensor_msgs/JointState` |

**Matemática:** reconstruye la velocidad de rueda desde el twist de `/odom` y la integra:

$$\omega_r = \frac{v + \omega\,L/2}{r}, \quad \omega_l = \frac{v - \omega\,L/2}{r}$$

$$\theta_{\text{rueda}}(t+\Delta t) = \theta_{\text{rueda}}(t) + \omega_{\text{rueda}}\,\Delta t$$

---

### 5.6  `control/controller.py` — PID de posición

**Rol:** lleva al robot al waypoint actual usando PID. Sólo se usa en Task 1 (en los experimentos Task 2 se reemplaza por el open_loop_driver).

**Contrato ROS:**

| Dirección | Topic | Tipo |
|-----------|-------|------|
| Sub | `/odom` | `nav_msgs/Odometry` |
| Sub | `/set_point` | `geometry_msgs/Point` |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` |
| Pub | `/next_point` | `std_msgs/Bool` (True cuando llega) |

**Matemática:**

$$e_d = \sqrt{(t_x-x)^2 + (t_y-y)^2}$$
$$e_a = \operatorname{atan2}(t_y - y, t_x - x) - \theta \quad \text{(normalizado a }(-\pi, \pi)\text{)}$$

$$\omega = k_{p,\omega}\,e_a + k_{i,\omega}\int e_a\,dt + k_{d,\omega}\frac{de_a}{dt}$$
$$v = k_v \cdot e_d \quad \text{si } |e_a| \le 0.2\text{ rad, sino } v = 0$$

Saturaciones: `v ∈ [0, 0.3] m/s`, `ω ∈ [−1.5, 1.5] rad/s`. La estrategia "alinear primero, avanzar después" produce esquinas limpias en figuras poligonales.

---

### 5.7  `control/trajectory_generator.py` — Generador de waypoints

**Rol:** secuencia los vértices de una figura geométrica al Controller.

**Contrato ROS:**

| Dirección | Topic | Tipo |
|-----------|-------|------|
| Sub | `/next_point` | `std_msgs/Bool` |
| Pub | `/set_point` | `geometry_msgs/Point` |
| Pub | `/visualization_marker_array` | `visualization_msgs/MarkerArray` |

**Protocolo:**

1. Espera 2 s al arrancar (deja que los nodos suban).
2. Publica el primer waypoint.
3. Cuando el Controller publica `next_point=True`, espera 2 s y publica el siguiente.

Figuras disponibles:

| Figura | Geometría |
|--------|-----------|
| `square` | cuadrado de 1.5 m de lado |
| `triangle` | triángulo casi equilátero, base 1.5 m |
| `hexagon` | hexágono regular, radio 1.0 m |

Marcadores en RViz: verde = waypoint actual, amarillo = pendientes.

---

### 5.8  Launch files

**`launch/puzzlebot_challenge5_launch.py` — Task 1 (sistema completo)**
Levanta 8 nodos: `robot_state_publisher`, `kinematic_sim`, `localisation`, `joint_state_publisher`, `coordinate_transform`, `controller`, `trajectory_generator`, `static_tf` y `rviz2`. Acepta los argumentos `shape`, `k_r`, `k_l`, `sample_time`.

**`launch/experiment_straight_launch.py` — Task 2 (recta)**
Reemplaza el Controller + Trajectory por el `open_loop_driver` con `(v=0.15, ω=0, T=6.67 s)`. Mantiene Localisation, TF y RViz.

**`launch/experiment_rotation_launch.py` — Task 2 (rotación)**
Igual al anterior pero con `(v=0, ω=π/2, T=4 s)`.

---

## 6. Cómo correrlo

> **Recomendado:** usa el launcher interactivo `run.sh`. Si prefieres comandos crudos, baja a la sección 6.3.

### 6.1 Launcher interactivo `run.sh`  ⭐

Un solo script con menús para compilar, lanzar Task 1, lanzar Task 2 y hacer diagnóstico ROS.

```bash
bash "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week5/Challenge/run.sh"
```

Al arrancar:
- Sourcea automáticamente `/opt/ros/humble/setup.bash` y tu `install/setup.bash`.
- Si no existe `install/`, compila el paquete antes de mostrar el menú.
- Crea la carpeta `output/` para guardar los PDFs del TF tree.

**Menú principal:**

```
MENÚ PRINCIPAL

  1)  Task 1   ·  Demostración con trayectoria + elipses
  2)  Task 2   ·  Calibrar k_r, k_l (experimentos)
  3)  Diagnóstico ROS  (nodos · topics · TF · rqt_graph)
  ── Mantenimiento ──
  4)  Recompilar el paquete
  5)  Limpieza total (rm build install log) + rebuild
  0)  Salir
```

#### 6.1.1 Submenú Task 1 — Demostración
Presets listos: cuadrado/triángulo/hexágono con ruido bajo, medio o alto. Opción 6 = personalizado.

#### 6.1.2 Submenú Task 2 — Experimentos de calibración
Recta de 1 m y rotación en sitio, cada uno con 3-4 presets de `(k_r, k_l)`. Opción 8 = personalizado.

#### 6.1.3 Submenú Diagnóstico ROS
| Opción | Comando subyacente | Para qué sirve |
|--------|---------------------|----------------|
| 1 | `ros2 node list` | Ver los 7 nodos activos |
| 2 | `ros2 topic list -t` | Topics con tipo de mensaje |
| 3 | `ros2 run tf2_tools view_frames` | **TF tree** → guarda PDF en `output/` y lo abre |
| 4 | `rqt_graph` | Grafo visual de nodos + topics (idéntico al diagrama del PDF) |
| 5 | `ros2 topic echo /odom --field pose.covariance` | Inspeccionar Σₖ en vivo |
| 6 | `ros2 topic hz` por cada topic clave | Verificar que todo corre a 50 Hz |
| 7 | Diagrama ASCII | Recordatorio rápido de la arquitectura |
| 8 | Abrir `output/` en el explorador | Acceder a PDFs generados |

### 6.2 Workflow recomendado para evidenciar el reto

**Terminal 1** — lanza el sistema:
```bash
./run.sh        # → 1 (Task 1) → 1 (Square default)
```

**Terminal 2** — captura evidencias en paralelo:
```bash
./run.sh        # → 3 (Diagnóstico)
                #   → 3  (genera TF tree PDF)
                #   → 4  (abre rqt_graph para screenshots)
                #   → 5  (imprime la covarianza)
```

### 6.3 Comandos crudos (si no quieres usar `run.sh`)

#### Compilar
```bash
cd "/home/alfonso/Documents/8 Semestre/manchester_bloque"
colcon build --packages-select puzzlebot_sim_w5 --paths \
    "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week5/Challenge/puzzlebot_sim"
source install/setup.bash
```

#### Task 1 — Demostración
```bash
ros2 launch puzzlebot_sim_w5 puzzlebot_challenge5_launch.py
ros2 launch puzzlebot_sim_w5 puzzlebot_challenge5_launch.py shape:=hexagon k_r:=0.10 k_l:=0.10
```
Figuras: `square` · `triangle` · `hexagon`

#### Task 2 — Experimentos
```bash
ros2 launch puzzlebot_sim_w5 experiment_straight_launch.py
ros2 launch puzzlebot_sim_w5 experiment_straight_launch.py k_r:=0.10 k_l:=0.10
ros2 launch puzzlebot_sim_w5 experiment_rotation_launch.py
ros2 launch puzzlebot_sim_w5 experiment_rotation_launch.py k_r:=0.15 k_l:=0.15
```

#### Inspeccionar la covarianza
```bash
ros2 topic echo /odom --field pose.covariance
```

#### Generar TF tree manualmente
```bash
cd "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/Week5/Challenge/output"
ros2 run tf2_tools view_frames
xdg-open frames_*.pdf
```

---

## 7. Metodología de calibración de `k_r`, `k_l`

1. **Experimento Recta (1 m):**
   - Lanza `experiment_straight_launch.py` con `k_r = k_l = 0.05` (default).
   - Mide el desplazamiento real del Puzzlebot físico al ejecutar el mismo comando.
   - Compara la pose final simulada vs la real → calcula el error (Δx, Δy).
   - Repite con 5-10 corridas para promediar.
   - Ajusta `k_r, k_l` hasta que **el 95 % de las corridas reales caigan dentro de la elipse 2σ** de la simulación.

2. **Experimento Rotación (≈1 vuelta):**
   - Mismo procedimiento, pero ahora compara el ángulo final θ.
   - Esto aísla el componente `σ_θθ` porque la traslación es casi nula.

3. **Criterio de buena calibración (test del 95 %):**
   ```
   k_r, k_l  óptimos  ⟺  (s_real − μ)ᵀ Σ⁻¹ (s_real − μ)  ≤  χ²₃,₀.₉₅ ≈ 7.81
   ```
   en el 95 % de los experimentos.

4. **Tip práctico:** si la elipse simulada es siempre **más pequeña** que la dispersión real → aumenta `k`. Si es exageradamente grande → bájalo.

---

## 8. Resumen de cambios vs Week 4

| Archivo | Cambio |
|---------|--------|
| `package.xml` | Nombre del paquete: `puzzlebot_sim_w5`, descripción y maintainer actualizados |
| `setup.py` | Paths internos reorganizados por dominio + nuevo entry point `part2_open_loop_driver` |
| `setup.cfg` | Rutas de scripts actualizadas al nuevo nombre |
| `resource/puzzlebot_sim_w5` | Renombrado para coincidir con el package name |
| Estructura interna | Reorganizada `part1/part2/part3` → `simulation/localisation/visualisation/control` |
| `localisation/localisation.py` | ⭐ Propagación Σₖ + parámetros `k_r, k_l` |
| `simulation/open_loop_driver.py` | Nodo helper para calibración (Task 2) |
| `launch/puzzlebot_challenge5_launch.py` | Launch principal con argumentos `k_r`, `k_l`, `sample_time` |
| `launch/experiment_straight_launch.py` | Experimento Task 2 — recta 1 m |
| `launch/experiment_rotation_launch.py` | Experimento Task 2 — rotación |
| `rviz/puzzlebot_covariance.rviz` | Config con "Odometry > Covariance" habilitado |
| `urdf/puzzlebot.urdf` | Meshes referenciados con `file://` (paths absolutos URL-encoded) |
| `Challenge/run.sh` | Launcher interactivo con menús de Task 1, Task 2 y diagnóstico ROS |
| `Challenge/output/` | Carpeta para evidencias (PDFs de TF tree) |
| Launches de Week 4 | Eliminados (no aplican al reto) |

---

## 9. Cumplimiento del enunciado — checklist

- [x] Matriz 3×3 de covarianza completada en el mensaje `/odom`
- [x] Propagación `Σₖ = H · Σₖ₋₁ · Hᵀ + Q` implementada **a mano** (sin librerías)
- [x] Mapeo correcto a `pose.covariance[36]` para que **RVIZ dibuje la elipse automáticamente**
- [x] Parámetros `k_r, k_l` parametrizables desde el launch para Task 2
- [x] Experimentos de calibración (recta + rotación) listos como launches independientes
- [x] Transformación TF `odom → base_footprint` publicada
- [x] EDOs resueltas con **Euler** (sin `scipy`, sin `odeint`)
- [x] Sólo NumPy + Python estándar
- [x] No hay teleop ni intervención humana en runtime
- [x] Sample time configurable (default `0.02 s` ≡ 50 Hz)

---

## 10. Notas de instalación y troubleshooting

Estos son los problemas que se encontraron al integrar el reto en la máquina de desarrollo y cómo se resolvieron. Útil si lo clonas en otra Ubuntu/ROS Humble.

### 10.1 La ruta del workspace contiene espacios (`8 Semestre`)
- **Síntoma:** `colcon` no encuentra el paquete cuando se le pasa una ruta relativa.
- **Fix:** usar **ruta absoluta** en `--paths`, o crear un workspace dedicado con symlink (`~/ros2_ws/src`).
- El `run.sh` ya pasa la ruta absoluta — no tienes que pensarlo.

### 10.2 URDF con `package://` no resolvía
- **Síntoma:** RViz mostraba `Could not load mesh resource`.
- **Causa:** combinación de `AMENT_PREFIX_PATH` con espacios + nombre de paquete renombrado.
- **Fix aplicado:** el URDF usa rutas absolutas `file://` URL-encoded:
  ```xml
  <mesh filename="file:///home/alfonso/Documents/8%20Semestre/.../Puzzlebot_Wheel.stl"/>
  ```

### 10.3 `rqt_graph` tronaba con `ImportError: DelimitedList`
- **Causa:** `pydot 4.0.1` (de `pip install --user`) es incompatible con `pyparsing 3.x`.
- **Fix:**
  ```bash
  pip3 uninstall -y pydot
  pip3 install "pydot>=3,<4"
  ```

### 10.4 `set -u` rompe scripts que sourcean `/opt/ros/humble/setup.bash`
- **Causa:** los scripts de ROS Humble usan variables sin inicializar internamente.
- **Fix aplicado en `run.sh`:** NO se usa `set -u`, y los `source` están envueltos en `|| true`.

---

## 11. Próximos pasos sugeridos

1. **Conectar al Puzzlebot real** y correr `experiment_straight_launch.py` varias veces, midiendo la pose final con cinta métrica o ArUco.
2. **Anotar (k_r, k_l, error)** para cada corrida → ajustar las ganancias.
3. **(Bonus)** Convertir el nodo en un **EKF** agregando una observación tipo LiDAR o ArUco para ver cómo la elipse se **encoge** al fusionar información.

---

> Manchester Robotics — *{Learn, Create, Innovate};*
