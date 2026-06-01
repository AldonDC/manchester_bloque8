#!/usr/bin/env bash
# =============================================================================
#  Final Challenge — Week 7
#  Manchester Robotics × NVIDIA · TE3003B
#
#  Launcher unificado para Part 1 (EKF Localisation) y Part 2 (Multi-waypoint
#  Reactive Navigation). Una sola terminal, comandos cortos.
#
#  Uso interactivo:
#      ./run.sh                  ← menú interactivo
#
#  Uso directo (CLI):
#      ./run.sh build            ← compila puzzlebot_ekf + puzzlebot_nav
#      ./run.sh part1            ← Part 1: EKF + ArUcos + RViz
#      ./run.sh part1-demo       ← Part 1 con demo automática (escenarios PDF)
#      ./run.sh part1-eval       ← Part 1: evaluación completa (RMSE + NEES)
#      ./run.sh part2            ← Part 2: 4 waypoints (Bug 2) en el laberinto
#      ./run.sh part2-bug0       ← Part 2: alternativa con Bug 0
#      ./run.sh part2-bug1       ← Part 2: Bug 1 (vuelta completa, robusto en cuartos)
#      ./run.sh part2-goto       ← Part 2: go-to-goal clásico (simple, recomendado PDF)
#      ./run.sh part2-integrated ← Part 2 + Opcional A + Opcional B (EKF+ArUcos+Nav)
#      ./run.sh part2-astar      ← Part 2 con A* + Catmull-Rom + Pure Pursuit (PRO)
#      ./run.sh move             ← teleop manual (con part1/part2 en otra terminal)
#      ./run.sh clean            ← rm build/install/log y recompila
#
#  Variables de entorno:
#      GPU=1 ./run.sh part2-bug1 ← usa NVIDIA RTX (DEFAULT — RViz/Gazebo a 60 fps)
#      GPU=0 ./run.sh ...        ← fallback software rendering (lento, debug)
# =============================================================================

WORKSPACE="/home/alfonso/Documents/8 Semestre/manchester_bloque"
PKG_EKF="puzzlebot_ekf"
PKG_NAV="puzzlebot_nav"
PKG_EKF_SRC="${WORKSPACE}/challenges/Week7/Challenge/part_1_ekf_localisation/puzzlebot_ekf"
PKG_NAV_SRC="${WORKSPACE}/challenges/Week7/Challenge/part_2_navigation/puzzlebot_nav"
PKG_GZ_SRC="${WORKSPACE}/challenges/Week5/Gazebo Simulator"

# ── Colores ──────────────────────────────────────────────────────────────────
C_RESET='\033[0m'; C_BOLD='\033[1m'; C_DIM='\033[2m'
C_RED='\033[0;31m'; C_GREEN='\033[0;32m'; C_YELLOW='\033[0;33m'
C_BLUE='\033[0;34m'; C_MAGENTA='\033[0;35m'; C_CYAN='\033[0;36m'; C_WHITE='\033[1;37m'

ok()   { echo -e "${C_GREEN}[ok]${C_RESET}    $1"; }
warn() { echo -e "${C_YELLOW}[warn]${C_RESET}  $1"; }
err()  { echo -e "${C_RED}[err]${C_RESET}   $1"; }
info() { echo -e "${C_CYAN}[info]${C_RESET}  $1"; }
section() { echo -e "\n${C_BLUE}${C_BOLD}── $1 ──${C_RESET}\n"; }
pause() { echo; read -r -p "$(echo -e ${C_DIM}Presiona Enter para continuar...${C_RESET})" _; }

banner() {
    clear
    echo -e "${C_CYAN}${C_BOLD}"
    cat << 'EOF'
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│    FINAL CHALLENGE · Week 7 · Manchester Robotics × NVIDIA               │
│    Part 1 (EKF Localisation) · Part 2 (Multi-waypoint Navigation)        │
│                                                                          │
│    Autor: Alfonso Diaz                                                   │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
EOF
    echo -e "${C_RESET}"
}

# ── Setup ────────────────────────────────────────────────────────────────────
clean_env_paths() {
    local cleaned_path=""
    IFS=':' read -ra ADDR <<< "$PATH"
    for i in "${ADDR[@]}"; do
        if [[ "$i" != *".pyenv"* && "$i" != *"miniconda"* && "$i" != *"anaconda"* ]]; then
            if [[ -z "$cleaned_path" ]]; then
                cleaned_path="$i"
            else
                cleaned_path="$cleaned_path:$i"
            fi
        fi
    done
    export PATH="$cleaned_path"
    export PYTHONNOUSERSITE=1
}

ensure_sourced() {
    clean_env_paths
    cd "$WORKSPACE" || { err "No accedo al workspace"; exit 1; }
    { source /opt/ros/humble/setup.bash; } 2>/dev/null || true
    if [[ -f "install/setup.bash" ]]; then
        { source install/setup.bash; } 2>/dev/null || true
        ok "Workspace sourceado"
    else
        warn "install/ no existe — compilando primero..."
        build_all
    fi
}

build_all() {
    section "Compilando puzzlebot_ekf + puzzlebot_nav + deps MCR2"
    cd "$WORKSPACE" || exit 1
    { source /opt/ros/humble/setup.bash; } 2>/dev/null || true
    colcon build \
        --packages-select "$PKG_EKF" "$PKG_NAV" puzzlebot_gazebo puzzlebot_description \
        --paths "$PKG_EKF_SRC" "$PKG_NAV_SRC" \
                "$PKG_GZ_SRC/puzzlebot_gazebo" "$PKG_GZ_SRC/puzzlebot_description"
    { source install/setup.bash; } 2>/dev/null || true
    ok "Compilación lista"
}

clean_build() {
    section "Limpieza total (build/ install/ log/) + rebuild"
    cd "$WORKSPACE" || exit 1
    rm -rf build install log
    build_all
}

# ── Kill procesos viejos antes de un nuevo launch ────────────────────────────
kill_leftovers() {
    pkill -9 -f "ros2 launch"          2>/dev/null
    pkill -9 -f "puzzlebot_ekf"        2>/dev/null
    pkill -9 -f "puzzlebot_nav"        2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "parameter_bridge"     2>/dev/null
    pkill -9 -f "ros_gz_bridge"        2>/dev/null
    pkill -9 -f "gz sim"               2>/dev/null
    pkill -9 -x  gz                    2>/dev/null
    pkill -9 -x  rviz2                 2>/dev/null
    pkill -9 -f "ruby.*gz"             2>/dev/null
    pkill -9 -f "gz-sim-gui"           2>/dev/null
    pkill -9 -f "ros_gz_sim"           2>/dev/null
    sleep 2
}

# ── Render env ───────────────────────────────────────────────────────────────
# Por defecto: software rendering vía llvmpipe (compatible con cualquier máquina
# pero MUY lento — 5-10 fps en Gazebo).
# Con GPU=1: usa NVIDIA PRIME render offload (laptops Optimus con NVIDIA + Intel).
# La tarjeta de Alfonso es RTX 4050 Laptop, así que con GPU=1 los FPS suben
# de ~6 a ~40-60 en RViz/Gazebo. Verificado con glxinfo:
#   sin variables  → Intel GT2
#   con __NV_*=1   → NVIDIA RTX 4050
# El system python (numpy 1.21 + cv2 4.5) se fuerza igual para no chocar
# con el pyenv numpy 2.
export_render_env() {
    # GPU=0 (default): software rendering Mesa/llvmpipe. Funciona SIEMPRE.
    # GPU=1: PRIME render offload a NVIDIA RTX. Requiere que tu sesión X
    #   tenga el provider NVIDIA cargado (verifica con `xrandr --listproviders`
    #   — debes ver "NVIDIA-0" o "modesetting" con sink offload Y source).
    #   Si NO está cargado (caso típico tras boot Intel-only), RViz/Gazebo
    #   crashea con "BadValue X_GLXCreateNewContext / Failed to create an
    #   OpenGL context". Solución: relogear con NVIDIA Prime activado vía
    #   `sudo prime-select nvidia` o el GUI de NVIDIA Settings.
    if [[ "${GPU:-0}" == "1" ]]; then
        # ── NVIDIA RTX (PRIME render offload) ──────────────────────────────
        export __NV_PRIME_RENDER_OFFLOAD=1
        export __GLX_VENDOR_LIBRARY_NAME=nvidia
        export __VK_LAYER_NV_optimus=NVIDIA_only
        unset LIBGL_ALWAYS_SOFTWARE 2>/dev/null || true
        unset GALLIUM_DRIVER        2>/dev/null || true
        unset MESA_LOADER_DRIVER_OVERRIDE 2>/dev/null || true
        export OGRE_RTT_MODE=Copy
        ok "GPU mode: NVIDIA RTX 4050 (PRIME offload)"
        # Verificación rápida: si el provider NVIDIA NO está en la sesión X,
        # advertimos antes de que crashee RViz.
        if ! xrandr --listproviders 2>/dev/null | grep -qi 'nvidia'; then
            warn "Tu sesión X no tiene provider NVIDIA. RViz/Gazebo crashearán."
            warn "Sal de sesión, elige 'Ubuntu on NVIDIA' en GDM, o ejecuta:"
            warn "    sudo prime-select nvidia && reboot"
            warn "Mientras tanto, usa GPU=0 ./run.sh ... (software rendering)."
        fi
    else
        # ── Software rendering (lento pero ESTABLE) ────────────────────────
        unset __NV_PRIME_RENDER_OFFLOAD 2>/dev/null || true
        unset __VK_LAYER_NV_optimus     2>/dev/null || true
        export __GLX_VENDOR_LIBRARY_NAME=mesa
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
        export MESA_LOADER_DRIVER_OVERRIDE=kms_swrast
        export OGRE_RTT_MODE=Copy
        ok "Render mode: software (Mesa llvmpipe) — estable"
    fi
    export QT_QPA_PLATFORM=xcb
    export PYTHONNOUSERSITE=1
    export PATH="/usr/bin:${PATH#"$HOME/.pyenv/shims:"}"
}

# ─────────────────────────────────────────────────────────────────────────────
# Launchers
# ─────────────────────────────────────────────────────────────────────────────

# ── Resumen del escenario fijo (ekf_arena.world) ─────────────────────────────
print_scenario_summary() {
    echo -e "  ${C_DIM}World:${C_RESET}   ekf_arena.world (laberinto 7×6 m)"
    echo -e "  ${C_DIM}Spawn:${C_RESET}   x=0.9  y=0.8  yaw=π/2  (corredor sur, mirando norte)"
    echo -e "  ${C_DIM}ArUcos:${C_RESET}  4 markers (id 0,1,2,3) en el muro divisorio central (y=2.0)"
    echo -e "                    id=1 (0.6,1.92) ◄ frente al robot al spawn"
    echo -e "                    id=2 (4.4,1.92) ◄ extremo este del corredor sur"
    echo -e "                    id=0 (0.6,2.08) ◄ extremo oeste del corredor norte"
    echo -e "                    id=3 (4.4,2.08) ◄ extremo este del corredor norte"
    echo
}

# Part 1 — EKF Localisation: Gazebo + laberinto + cámara + ArUcos + EKF + RViz
launch_part1() {
    section "PART 1 · EKF Localisation con ArUcos"
    print_scenario_summary
    info "Topics útiles:"
    echo "  /ekf/odom              ← pose estimada (EKF)"
    echo "  /ekf/covariance_ellipse ← elipse 99% en RViz"
    echo "  /aruco/observations    ← detecciones ArUco"
    echo "  /aruco/image           ← cámara con overlay (panel RViz)"
    echo "  /ground_truth          ← pose real del simulador"
    echo
    info "Limpiando procesos previos..."
    kill_leftovers
    export_render_env
    info "Lanzando... (Ctrl+C para detener)"
    ros2 launch "$PKG_EKF" ekf_sim_launch.py
}

# Part 1 con demo automática (escenarios del PDF: multi/no/partial marker)
launch_part1_demo() {
    section "PART 1 · EKF + Demo automática (escenarios del PDF)"
    print_scenario_summary
    info "Demo automática: el robot recorre 3 escenarios del PDF"
    echo "  1) multi-marker  · varios markers visibles a la vez"
    echo "  2) no-marker     · solo odometría (la elipse crece)"
    echo "  3) partial       · un marker entra/sale del FOV"
    echo
    info "Limpiando procesos previos..."
    kill_leftovers
    export_render_env
    info "Demo dura ~45 s. Graba RViz para el video. Ctrl+C para detener."
    ros2 launch "$PKG_EKF" ekf_sim_launch.py auto_demo:=true
}

# Part 1 · evaluación completa para el entregable del PDF.
# Lanza Part 1 con auto-demo y al terminar (Ctrl+C) el ekf_node imprime el
# resumen final con RMSE, NEES, conteos de updates/rejects por marker. Una
# sola ejecución cubre todo lo que pide la Part 1 del PDF.
launch_part1_eval() {
    section "PART 1 · EVALUACIÓN COMPLETA (RMSE + NEES + escenarios PDF)"
    print_scenario_summary
    info "Esta opción ejecuta lo que pide el PDF Part 1:"
    echo "  ✓ Detección ArUco + EKF predict/update"
    echo "  ✓ Transformaciones marker → cam → robot → world"
    echo "  ✓ Elipse de covarianza 99% en RViz"
    echo "  ✓ 3 escenarios automáticos (multi / no / partial marker)"
    echo "  ✓ Métricas en vivo cada 1 s en la terminal"
    echo "  ✓ Al cerrar con Ctrl+C: RMSE_xy, RMSE_yaw, NEES_mean,"
    echo "    NEES_in_band%, updates aceptados / rechazados por marker"
    echo
    info "Topics que puedes grabar en otra terminal:"
    echo "  ros2 topic echo /ekf/metrics  ← métricas instantáneas (CSV)"
    echo "  ros2 bag record /ekf/odom /ground_truth /aruco/observations /ekf/metrics"
    echo
    info "Lee docs/JUSTIFICATIONS.md para la defensa de cada decisión."
    echo
    info "Limpiando procesos previos..."
    kill_leftovers
    export_render_env
    warn "La demo dura ~45 s. PRESIONA Ctrl+C cuando termine para ver el resumen final."
    sleep 2
    ros2 launch "$PKG_EKF" ekf_sim_launch.py auto_demo:=true
}

# Part 2 — Multi-waypoint nav DENTRO del laberinto de Part 1
launch_part2() {
    local bug="${1:-bug2}"
    section "PART 2 · Multi-waypoint (${bug^^}) en el laberinto"
    print_scenario_summary
    info "Waypoints (trayectoria cerrada, en loop):"
    echo "  p0 (2.0, 0.5)  ← sur del muro divisorio, entre iv2 e iv3"
    echo "  p1 (4.7, 2.0)  ← corredor este, junto a la pared exterior"
    echo "  p2 (2.0, 3.5)  ← norte del muro divisorio, entre iv1 e iv4"
    echo "  p3 (-0.5, 2.5) ← corredor oeste, junto a la pared exterior"
    echo
    info "El robot circumnavega los 4 cuartos interiores usando ${bug^^}."
    info "Limpiando procesos previos..."
    kill_leftovers
    export_render_env
    info "Lanzando... (Ctrl+C para detener)"
    ros2 launch "$PKG_NAV" part2_in_maze_launch.py bug:="$bug"
}

# ── Part 2 + Opcional A + Opcional B (INTEGRACIÓN FULL) ─────────────────────
# Lanza el stack completo: Gazebo + Puzzlebot + LiDAR + Cámara + ArUco detector
# + EKF (Part 1) + Waypoint manager + Navegador (goto_goal o bug) + RViz.
# La nav usa /ekf/odom (pose corregida) en vez de /odom (encoders crudos):
# eso es lo que pide la Sección B opcional del PDF.
launch_part2_integrated() {
    local bug="${1:-goto_goal}"
    section "PART 2 INTEGRATED · Nav + EKF + ArUcos (Opcional B)"
    print_scenario_summary
    info "Stack completo según arquitectura del PDF:"
    echo "  [Waypoints]  → [${bug}]  → /cmd_vel → Gazebo"
    echo "  [LiDAR]      → [lidar_processor] → bug/d_front,left,right"
    echo "  [Camera]     → [aruco_detector] → /aruco/observations"
    echo "  [Encoders]   → [EKF (Part 1)] → /ekf/odom ← USA ESTO LA NAV"
    echo "  [ArUco map]  → EKF"
    echo
    info "Waypoints (trayectoria cerrada, en loop):"
    echo "  p0 (1.9, 0.0)  ← centro corredor sur, alineado al gap"
    echo "  p1 (5.2, 2.0)  ← corredor este, a altura del gap"
    echo "  p2 (1.9, 4.7)  ← centro corredor norte"
    echo "  p3 (-0.5, 2.0) ← corredor oeste"
    echo
    info "ArUco markers (mapa fijo de Part 1):"
    echo "  id=0,1,2,3 en el muro divisorio central (y=2.0)"
    info "Topics útiles para grabar en otra terminal:"
    echo "  ros2 topic echo /ekf/odom           ← pose corregida"
    echo "  ros2 topic echo /aruco/observations ← detecciones ArUco"
    echo "  ros2 topic echo /waypoint_manager/state"
    echo "  ros2 topic echo /bug/state          ← estado FSM"
    echo
    info "Limpiando procesos previos..."
    kill_leftovers
    export_render_env
    info "Lanzando... (Ctrl+C para detener)"
    ros2 launch "$PKG_NAV" part2_integrated_launch.py bug:="$bug"
}

# ── Part 2 con A* + Catmull-Rom + Pure Pursuit (enfoque SENIOR) ────────────
# Stack profesional sin BUG reactivo: planificación A* offline, suavizado
# Catmull-Rom, tracking con Pure Pursuit. Garantiza trayectoria válida
# desde el inicio porque conoce el mapa del laberinto.
launch_part2_astar() {
    section "PART 2 ASTAR · A* + Catmull-Rom + Pure Pursuit"
    print_scenario_summary
    info "Pipeline (enfoque senior, sin Bug reactivo):"
    echo "  [Mapa YAML]  → OccupancyGrid 140x120 (resolución 5cm)"
    echo "  [Waypoints]  → A* desde cero (8-connectividad, heap binario)"
    echo "  [Path crudo] → Catmull-Rom spline (suavizado C¹)"
    echo "  [Path suave] → Pure Pursuit tracker (lookahead 0.40m)"
    echo "  [EKF /ekf/odom] → pose corregida con ArUcos (Sección B)"
    echo
    info "Waypoints (v5, validados):"
    echo "  p0 (2.5, 0.5)  ← centro patio sur"
    echo "  p1 (5.2, 2.5)  ← corredor este (norte del divisor)"
    echo "  p2 (2.5, 3.5)  ← centro patio norte"
    echo "  p3 (-0.5, 1.5) ← corredor oeste (sur del divisor)"
    echo
    info "Topics útiles en otra terminal:"
    echo "  ros2 topic echo /planned_path   ← path A* en RViz"
    echo "  ros2 topic echo /occupancy_grid ← grid visualizable"
    echo "  ros2 topic echo /astar_nav/state ← FSM state"
    echo
    info "Limpiando procesos previos..."
    kill_leftovers
    export_render_env
    info "Lanzando... (Ctrl+C para detener)"
    ros2 launch "$PKG_NAV" part2_astar_launch.py
}

# ── Mover robot manualmente (en otra terminal con part1/part2 corriendo) ────
move_robot() {
    section "Mover robot manualmente"
    info "Necesitas Part 1 o Part 2 corriendo en otra terminal."
    echo "  1)  Avanzar"
    echo "  2)  Retroceder"
    echo "  3)  Girar izquierda"
    echo "  4)  Girar derecha"
    echo "  5)  Detener"
    echo "  0)  Volver"
    read -r -p "$(echo -e ${C_CYAN}▸ Opción: ${C_RESET})" opt
    case "$opt" in
        1) ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' ;;
        2) ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.15}}' ;;
        3) ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}' ;;
        4) ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: -0.5}}' ;;
        5) ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' ;;
        0) return ;;
        *) err "Opción inválida"; sleep 1 ;;
    esac
}

# ─────────────────────────────────────────────────────────────────────────────
# Menú principal
# ─────────────────────────────────────────────────────────────────────────────
main_menu() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}MENÚ PRINCIPAL${C_RESET}\n"
        echo -e "  ${C_BLUE}${C_BOLD}PARTE 1 · EKF Localisation${C_RESET}"
        echo -e "  ${C_MAGENTA}1)${C_RESET}  Ver EKF en vivo        ${C_DIM}·  Gazebo + ArUcos + elipse en RViz${C_RESET}"
        echo -e "  ${C_MAGENTA}2)${C_RESET}  ${C_GREEN}Evaluación completa${C_RESET}    ${C_DIM}·  RMSE + NEES + escenarios PDF ${C_GREEN}★ entregable${C_RESET}"
        echo
        echo -e "  ${C_BLUE}${C_BOLD}PARTE 2 · Navegación${C_RESET}"
        echo -e "  ${C_MAGENTA}3)${C_RESET}  4 waypoints (goto_goal) ${C_DIM}·  FSM con LiDAR, trayectoria cerrada${C_RESET}"
        echo -e "  ${C_MAGENTA}4)${C_RESET}  ${C_GREEN}INTEGRATED${C_RESET}             ${C_DIM}·  Nav + EKF + ArUcos ${C_GREEN}★ opcional B${C_RESET}"
        echo
        echo -e "  ${C_DIM}── Mantenimiento ──${C_RESET}"
        echo -e "  ${C_MAGENTA}9)${C_RESET}  Recompilar"
        echo -e "  ${C_MAGENTA}0)${C_RESET}  Salir"
        echo
        echo -e "  ${C_DIM}(CLI: part1, part1-demo, part2-bug0/bug1/bug2, part2-integrated — ./run.sh help)${C_RESET}"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Opción: ${C_RESET})" opt
        case "$opt" in
            1) launch_part1 ;;
            2) launch_part1_eval ;;
            3) launch_part2 goto_goal ;;
            4) launch_part2_integrated goto_goal ;;
            9) build_all; pause ;;
            0) echo -e "\n${C_GREEN}Bye.${C_RESET}\n"; exit 0 ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

# ── CLI no interactivo ───────────────────────────────────────────────────────
if [[ $# -ge 1 ]]; then
    sub="$1"; shift || true
    case "$sub" in
        build)        ensure_sourced; build_all ;;
        clean)        clean_build ;;
        part1)        ensure_sourced; launch_part1 ;;
        part1-demo)   ensure_sourced; launch_part1_demo ;;
        part1-eval)   ensure_sourced; launch_part1_eval ;;
        part2)        ensure_sourced; launch_part2 bug2 ;;
        part2-bug0)   ensure_sourced; launch_part2 bug0 ;;
        part2-bug1)   ensure_sourced; launch_part2 bug1 ;;
        part2-goto)   ensure_sourced; launch_part2 goto_goal ;;
        part2-integrated)   ensure_sourced; launch_part2_integrated goto_goal ;;
        part2-astar)        ensure_sourced; launch_part2_astar ;;
        part2-astar-fast)   ensure_sourced; export HEADLESS=1; launch_part2_astar ;;
        move)         ensure_sourced; move_robot ;;
        -h|--help|help)
            sed -n '1,28p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) err "Comando desconocido: $sub"; exit 1 ;;
    esac
    exit 0
fi

# ── Entry point interactivo ──────────────────────────────────────────────────
banner
ensure_sourced
sleep 1
main_menu
