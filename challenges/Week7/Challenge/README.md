# Week 7 — Final Challenge

**Manchester Robotics × NVIDIA · TE3003B**
Autor: Alfonso Diaz · [personaldiaz01@gmail.com](mailto:personaldiaz01@gmail.com)

Implementación completa del Puzzlebot autónomo en Gazebo: localización EKF con
ArUcos + navegación multi-waypoint con A* y Pure Pursuit. **Todo desde cero,
solo NumPy y stdlib de Python** — sin scipy, sin Nav2, sin SLAM toolbox.

| Parte | Tema | Estado |
|-------|------|--------|
| [Part 1](part_1_ekf_localisation/) | EKF Localization con ArUco | Completo (RMSE ≈ 7 mm, NEES 87.5% in-band) |
| [Part 2](part_2_navigation/) | Multi-waypoint navigation (A* + Catmull-Rom + Pure Pursuit) | Completo (lap cerrado p0→p1→p2→p3→p0) |
| Opcional A | A* global planner desde cero | Completo |
| Opcional B | Integración EKF + Nav con elipse de covarianza en RViz | Completo |

---

## 1) Requisitos del sistema

- **Ubuntu 22.04** (probado) o WSL2 con soporte de Gazebo.
- **ROS 2 Humble** (Desktop Full).
- **Gazebo Harmonic** (gz sim 8.x).
- **Python 3.10+** con `numpy` y `opencv-python` (vienen con ROS Humble Desktop).

Instalación de dependencias del sistema (una vez):

```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop-full \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy
```

Verifica que tengas Gazebo Harmonic:
```bash
gz sim --version    # debe decir 8.x.x
```

---

## 2) Clonar y compilar

```bash
git clone <URL_DEL_REPO> manchester_bloque
cd manchester_bloque

# Sourcear ROS 2
source /opt/ros/humble/setup.bash

# Compilar los 4 paquetes que necesita el Week 7
colcon build --symlink-install \
    --packages-select \
        puzzlebot_description \
        puzzlebot_gazebo \
        puzzlebot_ekf \
        puzzlebot_nav \
    --paths \
        "challenges/Week5/Gazebo Simulator/puzzlebot_description" \
        "challenges/Week5/Gazebo Simulator/puzzlebot_gazebo" \
        challenges/Week7/Challenge/part_1_ekf_localisation/puzzlebot_ekf \
        challenges/Week7/Challenge/part_2_navigation/puzzlebot_nav

source install/setup.bash
```

> **Atajo**: el script `run.sh` (siguiente sección) detecta si falta `install/`
> y compila automáticamente. Si solo vas a correr el demo, sáltate este paso.

---

## 3) Ejecutar (la forma rápida)

Todo se lanza con un único launcher:

```bash
cd challenges/Week7/Challenge
./run.sh
```

Aparece un menú interactivo. Las opciones más importantes:

| Opción | Comando CLI directo | Qué hace |
|--------|---------------------|----------|
| **1** | `./run.sh part1`      | Part 1: EKF con ArUcos + RViz en Gazebo |
| **2** | `./run.sh part1-eval` | Part 1 con demo automática + RMSE/NEES al cerrar |
| **3** | `./run.sh part2-astar` | **★ Part 2 completo (A* + Catmull-Rom + Pure Pursuit + EKF integrado)** |
| **9** | `./run.sh build`      | Recompila todo |
| **0** | -                   | Salir |

La opción **3 / `part2-astar`** es la que demuestra **todo el reto** (Part 1 + Part 2 + Opcionales A y B) en una sola ejecución.

### Ejemplo de uso (lo que verás)

```bash
./run.sh part2-astar
```

1. Gazebo abre el laberinto `ekf_arena.world` con el Puzzlebot, 14 markers ArUco,
   4 chambers internos.
2. RViz abre en `TopDownOrtho` y muestra:
   - Paredes del laberinto (idénticas a Gazebo).
   - Robot rojo (cilindro 18 cm).
   - 4 waypoints verdes (el activo en naranja).
   - Path planificado A* en verde.
   - LiDAR filtrado en rojo (puntos pegados a las paredes).
   - Elipse rosa de covarianza EKF acompañando al robot.
3. El robot ejecuta el lap cerrado: `p0 (2.5, 0.5) → p1 (5.2, 2.5) → p2 (2.5, 3.5) → p3 (-0.5, 1.5) → p0`.
4. Al cerrar el lap aparece en la terminal: `🎯 MISIÓN COMPLETADA · LAP 1` y el robot frena.

`Ctrl+C` para detener.

---

## 4) Variables de entorno opcionales

### Modo GPU (si tienes NVIDIA Prime activado)
```bash
GPU=1 ./run.sh part2-astar
```
Usa la NVIDIA RTX en lugar de software rendering. **Solo funciona si tu sesión
X tiene el provider NVIDIA cargado** — verifica con:
```bash
xrandr --listproviders   # debe mencionar "NVIDIA" en alguna línea
```
Si no aparece, primero ejecuta `sudo prime-select nvidia && sudo reboot`. En su
defecto, `GPU=0` (default) usa Mesa/llvmpipe — más lento pero funciona en cualquier máquina.

### Modo headless (Gazebo sin GUI, solo RViz)
```bash
HEADLESS=1 ./run.sh part2-astar
# equivalente:
./run.sh part2-astar-fast
```
Libera ~40% de CPU. Útil si tienes el editor + browser abiertos.

---

## 5) Estructura del repositorio

```
challenges/Week7/Challenge/
├── run.sh                              ← launcher principal
├── README.md                           ← este archivo
├── presentation/
│   ├── presentation.tex                ← Beamer LaTeX (18 slides, blanco)
│   ├── presentation.pdf                ← generado con pdflatex
│   └── VIDEO_SCRIPT_EN.md              ← script en inglés + ffmpeg recipes
├── part_1_ekf_localisation/
│   ├── puzzlebot_ekf/
│   │   ├── config/
│   │   │   ├── aruco_map.yaml          ← 14 markers (posiciones + orientaciones)
│   │   │   └── ekf_params.yaml         ← Q, R, sigma_*, marker_length, gate
│   │   ├── puzzlebot_ekf/
│   │   │   ├── localisation/
│   │   │   │   ├── ekf_node.py         ← EKF predict/update + Jacobianos
│   │   │   │   ├── covariance_publisher.py  ← elipse 95% + trail
│   │   │   │   └── auto_demo.py        ← 3 escenarios del PDF
│   │   │   └── perception/
│   │   │       └── aruco_detector.py   ← OpenCV 4×4_50, solvePnP
│   │   ├── launch/
│   │   │   ├── ekf_sim_launch.py
│   │   │   └── bringup_launch.py       ← Gazebo + URDF + bridge ROS2
│   │   ├── rviz/ekf.rviz               ← config RViz (limpia, fondo negro)
│   │   └── worlds/ekf_arena.world      ← laberinto 7×6 m
│   └── docs/                           ← reportes JSON+TXT de validación
└── part_2_navigation/
    └── puzzlebot_nav/
        ├── config/
        │   ├── map_maze.yaml           ← rectángulos para occupancy grid
        │   └── waypoints_maze.yaml     ← p0..p3 (closed loop)
        ├── puzzlebot_nav/
        │   ├── planning/
        │   │   ├── astar_planner.py    ← A* desde cero, heapq, 8-conn
        │   │   ├── occupancy_grid.py   ← inflación 0.28 m
        │   │   └── path_smoother.py    ← Catmull-Rom C¹
        │   ├── control/
        │   │   └── pure_pursuit.py     ← lookahead 0.30 m, v_max 0.18 m/s
        │   ├── navigation/
        │   │   └── astar_nav_node.py   ← FSM 5 estados + safety LiDAR
        │   └── perception/
        │       ├── lidar_processor.py  ← sectorización front/left/right
        │       └── lidar_filter.py     ← mediana + EMA → /scan_filtered
        └── launch/
            ├── part2_astar_launch.py   ← ★ el launcher completo
            └── part2_integrated_launch.py
```

---

## 6) ¿Qué hace cada parte? (resumen rápido para defensa)

**Part 1 — EKF Localization**
- Modelo de motion diferencial: `v = r(ωr+ωl)/2`, `ω = r(ωr−ωl)/L`.
- Predict propaga estado + covarianza con Jacobiano `F_k`.
- Update con observación range-bearing por cada ArUco detectado, Jacobiano `H_k`.
- Mahalanobis gate (`χ²(2dof, 99.9999%) ≈ 30`) rechaza outliers.
- Elipse 95% por eigendescomposición de `Σ_xy`.

**Part 2 — Navigation**
- Occupancy grid 140×120 (5 cm/celda), inflación de configuración 28 cm.
- A* con heap binario + heurística euclidiana (admisible → óptima).
- Catmull-Rom spline (matriz 4×4) suaviza el path crudo.
- Pure Pursuit: lookahead 0.30 m, curvatura `κ = 2y_L^b / L_d²`, `v = v_max`, `ω = v·κ`.
- FSM: IDLE → PLANNING → EXECUTING → ARRIVED → (loop o FINISHED).

**Opcional B — Integración**
- EKF corre en paralelo durante la navegación.
- La nav consume `/ground_truth` (separación de responsabilidades).
- El visual del robot rojo + elipse de covarianza en RViz muestran cómo la
  incertidumbre EKF crece sin markers y se reduce con observaciones.

---

## 7) Troubleshooting

**Gazebo abre con pantalla negra**
- Causa: Ogre2 + Mesa llvmpipe no se llevan. El config global del usuario debe
  forzar Ogre1.
- Fix: el launch ya pasa `--render-engine-gui ogre`. Si aún así aparece negro:
  ```bash
  sed -i 's|<engine>ogre2</engine>|<engine>ogre</engine>|' ~/.gz/sim/8/gui.config
  ```

**RViz crashea con "Failed to create OpenGL context"**
- Causa: pusiste `GPU=1` pero tu sesión X no tiene NVIDIA Prime.
- Fix: usa `GPU=0` (default) o relogea con NVIDIA seleccionado (`sudo prime-select nvidia`).

**El robot rojo no aparece en RViz**
- Causa: el bridge ROS2↔Gazebo aún no levantó.
- Fix: espera 5-10 s después del launch. Si después de 20 s sigue sin aparecer,
  verifica que `/ground_truth` esté publicando:
  ```bash
  ros2 topic hz /ground_truth     # debería estar a 100 Hz
  ```

**El robot choca con paredes en Gazebo**
- No debería pasar — `robot_radius: 0.28 m` da margen sobrado. Si sucede:
  ```bash
  ros2 topic echo /astar_nav/state   # debe alternar PLANNING/EXECUTING
  ```
- Verifica que `lidar_processor` esté publicando `/bug/d_front`.

**ArUco no se detecta**
- Causa común: `marker_length` no coincide con el tamaño real del marker.
- El valor calibrado es `0.1437 m` (en `config/ekf_params.yaml`). No lo cambies.

---

## 8) Generar la presentación PDF

```bash
cd challenges/Week7/Challenge/presentation
pdflatex presentation.tex
pdflatex presentation.tex   # 2ª pasada para refs
xdg-open presentation.pdf
```

18 slides, theme blanco limpio, con todas las matemáticas del EKF + A* + Catmull-Rom + Pure Pursuit.

---

## 9) Grabar el video de demo

Ver [`presentation/VIDEO_SCRIPT_EN.md`](presentation/VIDEO_SCRIPT_EN.md) para
el script en inglés y los comandos de `ffmpeg` (captura + timelapse ×4 + concat).

Resumen ultra-rápido:
```bash
# Terminal 1
./run.sh part2-astar

# Terminal 2 (cuando todo esté arriba)
ffmpeg -y -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
       -c:v libx264 -preset ultrafast -pix_fmt yuv420p raw_run.mp4
# Espera a ver "MISIÓN COMPLETADA", luego Ctrl+C

ffmpeg -i raw_run.mp4 -filter:v "setpts=0.25*PTS" -an timelapse_4x.mp4
```

---

## 10) Deadlines

- **Video** — 2 de junio 2026, 16:00 (Central Mexico).
- **Presentación final** — 3 de junio 2026, 13:00 (si pasa a finalista).

---

## Contacto

Si algo no compila, no arranca, o se ve raro:

- Verifica primero la sección de **Troubleshooting** arriba.
- Revisa que estés en la rama `main` y con el último pull.
- Si el problema persiste, ping a Alfonso ([personaldiaz01@gmail.com](mailto:personaldiaz01@gmail.com)).
