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
#      ./run.sh part2            ← Part 2: 4 waypoints en el laberinto
#      ./run.sh clean            ← rm build/install/log y recompila
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
ensure_sourced() {
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

# ── Render env (lo que hace Week 6 funcionar en esta máquina) ────────────────
# Software rendering llvmpipe vía Mesa. Hay que exportarlas en el shell padre
# ANTES del ros2 launch para que Gazebo + RViz hereden la config.
# También fuerza el system python (apt) para que numpy 1.21 + cv2 4.5
# no choquen con el pyenv numpy 2.
export_render_env() {
    unset __NV_PRIME_RENDER_OFFLOAD 2>/dev/null || true
    unset __VK_LAYER_NV_optimus     2>/dev/null || true
    export __GLX_VENDOR_LIBRARY_NAME=mesa
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    export MESA_LOADER_DRIVER_OVERRIDE=kms_swrast
    export OGRE_RTT_MODE=Copy
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
        echo -e "  ${C_MAGENTA}1)${C_RESET}  ${C_BOLD}Part 1${C_RESET}            ·  EKF Localisation (manual)"
        echo -e "  ${C_MAGENTA}2)${C_RESET}  ${C_BOLD}Part 1 + Demo${C_RESET}     ·  EKF con demo automática (escenarios PDF)"
        echo -e "  ${C_MAGENTA}3)${C_RESET}  ${C_BOLD}Part 2 · Bug 2${C_RESET}    ·  4 waypoints en el laberinto"
        echo -e "  ${C_MAGENTA}4)${C_RESET}  ${C_BOLD}Part 2 · Bug 0${C_RESET}    ·  alternativa más simple"
        echo
        echo -e "  ${C_MAGENTA}5)${C_RESET}  Mover robot manualmente"
        echo
        echo -e "  ${C_DIM}── Mantenimiento ──${C_RESET}"
        echo -e "  ${C_MAGENTA}6)${C_RESET}  Recompilar"
        echo -e "  ${C_MAGENTA}7)${C_RESET}  Limpieza total + rebuild"
        echo
        echo -e "  ${C_MAGENTA}0)${C_RESET}  Salir"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Opción: ${C_RESET})" opt
        case "$opt" in
            1) launch_part1 ;;
            2) launch_part1_demo ;;
            3) launch_part2 bug2 ;;
            4) launch_part2 bug0 ;;
            5) move_robot ;;
            6) build_all; pause ;;
            7) clean_build; pause ;;
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
        part2)        ensure_sourced; launch_part2 bug2 ;;
        part2-bug0)   ensure_sourced; launch_part2 bug0 ;;
        -h|--help|help)
            sed -n '1,17p' "$0" | sed 's/^# \{0,1\}//'
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
