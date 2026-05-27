# Final Challenge — Estado del proyecto

## ✅ Funciona

### Part 1 — EKF Localisation
- `puzzlebot_ekf` ROS 2 package builds limpio.
- 4 executables: `aruco_detector`, `ekf_node`, `covariance_publisher`, `world_visualizer`.
- 3 launches: `bringup_launch.py`, `ekf_only_launch.py`, `ekf_sim_launch.py`.
- World **simple** `ekf_arena.world`: piso de madera (puzzlebot_base) + 4 paredes + 4 ArUcos + 2 obstáculos azul/rojo.
- ArUco map local (`models/aruco_marker_0..3`) con SDF limpio (sin `normal_map` espurio, sin `<script>` legacy).
- EKF estable: covariance clamp + correction step saturation + sanity gate en detector.
- RViz config completa: TopDownOrtho, RobotModel, EKF Odom + covariance ellipse, ArUco map labels, camera debug feed.

### Part 2 — Multi-Waypoint Nav
- `puzzlebot_nav` ROS 2 package builds limpio (basado 1:1 en Week 6).
- 9 executables: bug0, bug2, controller, goal_publisher, **waypoint_manager** (nuevo), localisation, etc.
- `nav_multipoint_launch.py` chains ≥4 waypoints en loop cerrado.
- `config/waypoints.yaml` editable runtime.

### Estructura limpia
```
Week7/Challenge/
├── README.md                       (guía global)
├── STATUS.md                       (este archivo)
├── run.sh                          (launcher interactivo estilo Week 6)
├── part_1_ekf_localisation/
│   ├── README.md
│   └── puzzlebot_ekf/
│       ├── puzzlebot_ekf/{perception,localisation,map,utils}/
│       ├── config/{ekf_params,aruco_map}.yaml
│       ├── launch/{bringup,ekf_only,ekf_sim}_launch.py
│       ├── worlds/ekf_arena.world
│       ├── models/aruco_marker_{0..3}/    (locales, limpios)
│       ├── rviz/ekf.rviz
│       └── calibration/{saveImages,CameraCal}.py
├── part_2_navigation/
│   ├── README.md
│   └── puzzlebot_nav/
│       ├── puzzlebot_nav/{navigation,control,perception,localisation}/
│       │   └── navigation/waypoint_manager.py    (nuevo)
│       ├── config/waypoints.yaml
│       └── launch/nav_multipoint_launch.py
└── part_optional_integration/      (skeleton, no implementado aún)
```

## ⚠️ Conocido (limitación de la máquina, no del código)

### Robot Puzzlebot no se renderiza VISUALMENTE en la viewport de Gazebo
- **Síntoma**: solo se ve el texto `Puzzlebot` flotando; las mallas STL del chasis/ruedas/Jetson no se dibujan.
- **Causa raíz** (confirmada por `~/.gz/rendering/ogre2.log`):
  ```
  OGRE EXCEPTION: eglQueryDeviceStringEXT not found. EGL driver too old.
  ```
  `libegl1` de Ubuntu 22.04 es de 2018 y le falta esa función para los PBuffers offscreen que Ogre2 necesita para sensores de cámara.
- **Lo que SÍ funciona pese al bug**:
  - El robot está físicamente en la escena: publica `/joint_states`, `/wr`, `/wl`, `/camera/image_raw`, `/camera/camera_info`, `/scan` y `/ground_truth`.
  - El EKF recibe odometría y observaciones ArUco normalmente.
  - **RViz muestra el robot completo** con mallas (porque RViz usa su propio render Ogre1 vía glX).
- **Workaround aplicado**: software-rendering vía llvmpipe (exportado en `run.sh`) — funciona para Week 6 con worlds simples, pero al combinar con el laberinto plywood (DAE Collada `<blinn>` legacy + 47 paneles) la cascade de excepciones de Ogre2 vuelve a tumbar el visual del robot. Por eso `ekf_arena.world` ahora es la versión simple sin maze.

### Implicación práctica para el reto
**Para el video del Final Challenge, grabar RViz** (donde se ve todo: robot, trayectoria, ArUcos, covariance ellipse evolving). Gazebo sirve como backend de física y sensores, RViz como vista canónica.

## 📋 Pendiente

- [ ] Probar Part 2 end-to-end (`./run.sh` → 2 → bug_medium con bug2). Confirmar que el waypoint_manager avanza por los 4 puntos y cierra el loop.
- [ ] Docs:
  - `part_1_ekf_localisation/docs/observation_models.md` — comparativa range+bearing vs cartesian.
  - `part_1_ekf_localisation/docs/metrics.md` — RMSE vs `/ground_truth`, ATE.
- [ ] Outline del video 3-4 min (inglés) — qué es Kalman, cómo implementamos, retos enfrentados, comparativas.
- [ ] Part 3 (opcional): supervisor que combine EKF + waypoint nav.

## 🎬 Cómo grabar el video

1. Ejecuta `./run.sh` → 1 (Part 1) → 1 (sim completo)
2. Espera a que RViz cargue. Grabarás la ventana de RViz.
3. En otra terminal: `ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.3}}'`
4. En RViz verás:
   - Robot Puzzlebot completo (con mallas)
   - Flecha roja del EKF + elipse magenta de covarianza
   - 4 ArUcos blancos con labels `id=0..3`
   - Feed de cámara con detección dibujada en el panel inferior izquierdo
5. Para Part 2: `./run.sh` → 2 → bug_medium con bug2 → grabar RViz mostrando la trayectoria cerrada de 4 waypoints.

## 🔧 Cómo recompilar y limpiar

```bash
# Build incremental
./challenges/Week7/Challenge/run.sh build

# Clean rebuild
./challenges/Week7/Challenge/run.sh clean

# Menú interactivo
./challenges/Week7/Challenge/run.sh
```
