#!/usr/bin/env bash
# =============================================================================
#  Mini Challenge · Reactive Navigation (Bug 0 / Bug 2)
#  Launcher interactivo — Manchester Robotics · Week 6
# =============================================================================
#
# Nota: NO usamos `set -u` porque los scripts de ROS Humble
# referencian variables no inicializadas internamente.

WORKSPACE="/home/alfonso/Documents/8 Semestre/manchester_bloque"
PKG="puzzlebot_bug_w6"
PKG_SRC="${WORKSPACE}/challenges/Week6/Challenge/puzzlebot_bug"
PKG_GZ_SRC="${WORKSPACE}/challenges/Week5/Gazebo Simulator"
OUTPUT_DIR="${WORKSPACE}/challenges/Week6/Challenge/output"

# ── Colores ──────────────────────────────────────────────────────────────────
C_RESET='\033[0m'; C_BOLD='\033[1m'; C_DIM='\033[2m'
C_RED='\033[0;31m'; C_GREEN='\033[0;32m'; C_YELLOW='\033[0;33m'
C_BLUE='\033[0;34m'; C_MAGENTA='\033[0;35m'; C_CYAN='\033[0;36m'; C_WHITE='\033[1;37m'

# ── Helpers ──────────────────────────────────────────────────────────────────
banner() {
    clear
    echo -e "${C_CYAN}${C_BOLD}"
    cat << 'EOF'
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│    MINI CHALLENGE · REACTIVE NAVIGATION (Bug 0 / Bug 2)                  │
│                                                                          │
│    Week 6 · Manchester Robotics × NVIDIA                                 │
│    Autor: Alfonso Diaz                                                   │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
EOF
    echo -e "${C_RESET}"
}

section() { echo -e "\n${C_BLUE}${C_BOLD}── $1 ──${C_RESET}\n"; }
ok()   { echo -e "${C_GREEN}[ok]${C_RESET}    $1"; }
warn() { echo -e "${C_YELLOW}[warn]${C_RESET}  $1"; }
err()  { echo -e "${C_RED}[err]${C_RESET}   $1"; }
info() { echo -e "${C_CYAN}[info]${C_RESET}  $1"; }

ensure_dirs() { mkdir -p "$OUTPUT_DIR"; }

ensure_sourced() {
    cd "$WORKSPACE" || { err "No se puede acceder al workspace"; exit 1; }
    { source /opt/ros/humble/setup.bash; } 2>/dev/null || true
    ok "ROS Humble sourceado"
    if [[ -f "install/setup.bash" ]]; then
        { source install/setup.bash; } 2>/dev/null || true
        ok "Workspace local sourceado"
    else
        warn "No existe install/. Compilando primero..."
        build_pkg
    fi
}

build_pkg() {
    section "Compilando ${PKG} + dependencias del simulador MCR2"
    cd "$WORKSPACE" || exit 1
    # Construimos: nuestro paquete + puzzlebot_gazebo + puzzlebot_description
    colcon build \
        --packages-select "$PKG" puzzlebot_gazebo puzzlebot_description \
        --paths "$PKG_SRC" "$PKG_GZ_SRC/puzzlebot_gazebo" "$PKG_GZ_SRC/puzzlebot_description"
    { source install/setup.bash; } 2>/dev/null || true
    ok "Compilación lista"
}

clean_build() {
    section "Limpiando build / install / log"
    cd "$WORKSPACE" || exit 1
    rm -rf build install log
    ok "Limpio. Recompilando..."
    build_pkg
}

pause() { echo; read -r -p "$(echo -e ${C_DIM}Presiona Enter para continuar...${C_RESET})" _; }

# ── Lanzadores ───────────────────────────────────────────────────────────────

kill_leftovers() {
    # Mata cualquier instancia previa de Gazebo / RViz / nodos del bug.
    # Sin esto, los timestamps de robot_state_publisher se desincronizan
    # y RViz hace "Detected jump back in time" continuamente.
    pkill -9 -f "ros2 launch"  2>/dev/null
    pkill -9 -f "puzzlebot_bug" 2>/dev/null
    pkill -9 -f "static_transform_publisher" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "parameter_bridge" 2>/dev/null
    pkill -9 -f "ros_gz_bridge" 2>/dev/null
    pkill -9 -f "gz sim"  2>/dev/null
    pkill -9 -x  gz       2>/dev/null
    pkill -9 -x  rviz2    2>/dev/null
    # Procesos relacionados con el contexto OpenGL/Qt que a veces sobreviven
    # al pkill de "gz sim" y bloquean el driver NVIDIA en el siguiente launch
    # (síntoma: "QGLXContext: Failed to create dummy context" + abort).
    pkill -9 -f "ruby.*gz"        2>/dev/null
    pkill -9 -f "gz-sim-gui"      2>/dev/null
    pkill -9 -f "ros_gz_sim"      2>/dev/null
    # 3s: tiempo para que el driver NVIDIA libere el contexto GL viejo.
    # Con 2s a veces el siguiente lanzamiento aún veía contextos colgados.
    sleep 3
}

ensure_gpu_ready() {
    # Pre-arranca el driver NVIDIA si está dormido. En híbridas Intel+NVIDIA
    # con PRIME render-offload (los launches de Week 6 usan esto), la GPU
    # puede estar en estado D3cold tras un kill duro de Gazebo; el siguiente
    # eglInit puede crashear porque encuentra el dispositivo "presente pero
    # sin contexto". `nvidia-smi` despierta el driver sin renderizar nada.
    if command -v nvidia-smi >/dev/null 2>&1; then
        nvidia-smi -L >/dev/null 2>&1 || true
    fi
}

launch_bug() {
    # Uso:  launch_bug <algo> <world> <tx> <ty> [sx] [sy] [syaw]
    # Si se omiten sx/sy/syaw, el robot nace en (0,0,0).
    local algo="$1" world="$2" tx="$3" ty="$4"
    local sx="${5:-0.0}" sy="${6:-0.0}" syaw="${7:-0.0}"
    section "${algo^^} · world=${world}  start=(${sx}, ${sy}, ${syaw})  goal=(${tx}, ${ty})"
    info "Limpiando procesos previos..."
    kill_leftovers
    info "Despertando driver gráfico..."
    ensure_gpu_ready

    # Forzar el uso EXCLUSIVO de la iGPU Intel.
    #
    # Diagnóstico: en esta máquina el driver NVIDIA está en mal estado
    # (`nvidia-smi` reporta "No devices were found", forzar GLX_VENDOR=nvidia
    # tira BadValue). La iGPU Intel funciona bien. Mientras NVIDIA no se
    # arregle, usar Intel es lo único que carga RViz y Gazebo de forma
    # estable.
    #
    # Si en el futuro NVIDIA se reinstala/configura bien, basta con
    # comentar este bloque y descomentar el de PRIME render-offload de
    # commits anteriores. Para verificar el estado del driver:
    #     nvidia-smi -L
    #     glxinfo | grep "OpenGL renderer"
    unset __NV_PRIME_RENDER_OFFLOAD
    unset __GLX_VENDOR_LIBRARY_NAME
    unset __VK_LAYER_NV_optimus
    export __GLX_VENDOR_LIBRARY_NAME=mesa
    # Modos tolerantes para Ogre1 (RViz) y Ogre2 (Gazebo).
    export OGRE_RTT_MODE=Copy
    export QT_QPA_PLATFORM=xcb

    info "Ctrl+C para detener"
    sleep 1
    ros2 launch "$PKG" "${algo}_launch.py" \
        world:="$world" \
        x:="$sx" y:="$sy" yaw:="$syaw" \
        target_x:="$tx" target_y:="$ty"
}

# ── Diagnóstico ROS ──────────────────────────────────────────────────────────

show_nodes()   { section "Nodos activos";        ros2 node list; pause; }
show_topics()  { section "Topics publicados";    ros2 topic list -t; pause; }
show_state()   {
    section "Estado actual del Bug"
    info "Imprimiendo /bug/state — Ctrl+C para detener"
    ros2 topic echo /bug/state
}

show_distances() {
    section "Distancias del LiDAR (sectores)"
    info "Imprimiendo /bug/d_front,d_left,d_right — Ctrl+C para detener"
    paste \
        <(ros2 topic echo /bug/d_front --no-arr 2>/dev/null) \
        <(ros2 topic echo /bug/d_left  --no-arr 2>/dev/null) \
        <(ros2 topic echo /bug/d_right --no-arr 2>/dev/null) \
        | head -20
    pause
}

show_tf_tree() {
    section "Árbol de Transformadas (TF tree)"
    ensure_dirs
    local stamp; stamp="$(date +%Y%m%d_%H%M%S)"
    local out_pdf="${OUTPUT_DIR}/tf_tree_${stamp}.pdf"
    local tmpdir; tmpdir="$(mktemp -d)"
    pushd "$tmpdir" > /dev/null || return 1
    info "Capturando frames durante ~5 s..."
    timeout 8 ros2 run tf2_tools view_frames 2>&1 | tail -3
    popd > /dev/null || return 1
    local gen; gen="$(ls -t "$tmpdir"/frames*.pdf 2>/dev/null | head -1)"
    if [[ -n "$gen" ]]; then
        mv "$gen" "$out_pdf"
        ok "Guardado: $out_pdf"
        command -v xdg-open >/dev/null && xdg-open "$out_pdf" 2>/dev/null &
    else
        err "No se generó. ¿Está corriendo el launch?"
    fi
    rm -rf "$tmpdir"
    pause
}

show_rqt_graph() {
    section "Grafo de nodos (rqt_graph)"
    info "Abriendo en ventana separada..."
    rqt_graph &
    pause
}

open_output_dir() {
    section "Carpeta output/"
    ensure_dirs
    echo "  ${OUTPUT_DIR}"; echo
    ls -lh "$OUTPUT_DIR" 2>/dev/null || true
    command -v xdg-open >/dev/null && xdg-open "$OUTPUT_DIR" 2>/dev/null &
    pause
}

# ── Menús ────────────────────────────────────────────────────────────────────

menu_task1() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}TASK 1 · Bug 0 — Demostración${C_RESET}\n"
        echo -e "  ${C_DIM}── Escenarios canónicos ──${C_RESET}"
        echo "  1)  bug_easy        · 1 obstáculo                       (start 0,0   → goal 3.0,  0.0)"
        echo "  2)  bug_medium      · 2 obstáculos en cascada            (start 0,0   → goal 4.0,  0.0)"
        echo "  3)  bug_hard        · forma de U (Bug 0 puede fallar)    (start -2.5,0 → goal 2.5, 0.0)"
        echo
        echo -e "  ${C_DIM}── Escenarios extendidos (modelos MCR2 · Week 5) ──${C_RESET}"
        echo "  4)  bug_office      · cuarto cerrado + tabique central   (start -1.3,-1.5 → goal 1.3, 1.5)"
        echo "  5)  bug_arena       · ladrillos dispersos + bolsillo 'C' (start 0,0   → goal 2.4, 2.0)"
        echo "  6)  bug_hard_plus   · U con ladrillos en el approach     (start -2.5,0 → goal 2.5, 0.0)"
        echo
        echo -e "  ${C_DIM}── Otros ──${C_RESET}"
        echo "  7)  obstacle_avoidance_1.world (MCR2 default)"
        echo "  8)  Personalizado (elige world + start + goal)"
        echo "  0)  Volver"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) launch_bug bug0 bug_easy.world      3.0  0.0   0.0  0.0 0.0 ;;
            2) launch_bug bug0 bug_medium.world    4.0  0.0   0.0  0.0 0.0 ;;
            3) launch_bug bug0 bug_hard.world      2.5  0.0  -2.5  0.0 0.0 ;;
            4) launch_bug bug0 bug_office.world    1.3  1.5  -1.3 -1.5 0.0 ;;
            5) launch_bug bug0 bug_arena.world     2.4  2.0   0.0  0.0 0.0 ;;
            6) launch_bug bug0 bug_hard_plus.world 2.5  0.0  -2.5  0.0 0.0 ;;
            7) launch_bug bug0 obstacle_avoidance_1.world 1.5 1.2 0.0 0.0 0.0 ;;
            8)
                read -r -p "  World: "    w
                read -r -p "  start_x: "  sx
                read -r -p "  start_y: "  sy
                read -r -p "  start_yaw: " syaw
                read -r -p "  target_x: " a
                read -r -p "  target_y: " b
                launch_bug bug0 "${w:-bug_easy.world}" "${a:-3.0}" "${b:-0.0}" \
                                "${sx:-0.0}" "${sy:-0.0}" "${syaw:-0.0}"
                ;;
            0) return ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

menu_task2() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}TASK 2 · Bug 2 — Demostración${C_RESET}\n"
        echo -e "  ${C_DIM}── Escenarios canónicos ──${C_RESET}"
        echo "  1)  bug_easy        · 1 obstáculo                       (start 0,0   → goal 3.0,  0.0)"
        echo "  2)  bug_medium      · 2 obstáculos en cascada            (start 0,0   → goal 4.0,  0.0)"
        echo "  3)  bug_hard        · forma de U (Bug 2 la resuelve)     (start -2.5,0 → goal 2.5, 0.0)"
        echo
        echo -e "  ${C_DIM}── Escenarios extendidos (modelos MCR2 · Week 5) ──${C_RESET}"
        echo "  4)  bug_office      · cuarto cerrado + tabique central   (start -1.3,-1.5 → goal 1.3, 1.5)"
        echo "  5)  bug_arena       · ladrillos dispersos + bolsillo 'C' (start 0,0   → goal 2.4, 2.0)"
        echo "  6)  bug_hard_plus   · U con ladrillos en el approach     (start -2.5,0 → goal 2.5, 0.0)"
        echo
        echo -e "  ${C_DIM}── Otros ──${C_RESET}"
        echo "  7)  obstacle_avoidance_1.world (MCR2 default)"
        echo "  8)  Personalizado (elige world + start + goal)"
        echo "  0)  Volver"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) launch_bug bug2 bug_easy.world      3.0  0.0   0.0  0.0 0.0 ;;
            2) launch_bug bug2 bug_medium.world    4.0  0.0   0.0  0.0 0.0 ;;
            3) launch_bug bug2 bug_hard.world      2.5  0.0  -2.5  0.0 0.0 ;;
            4) launch_bug bug2 bug_office.world    1.3  1.5  -1.3 -1.5 0.0 ;;
            5) launch_bug bug2 bug_arena.world     2.4  2.0   0.0  0.0 0.0 ;;
            6) launch_bug bug2 bug_hard_plus.world 2.5  0.0  -2.5  0.0 0.0 ;;
            7) launch_bug bug2 obstacle_avoidance_1.world 1.5 1.2 0.0 0.0 0.0 ;;
            8)
                read -r -p "  World: "    w
                read -r -p "  start_x: "  sx
                read -r -p "  start_y: "  sy
                read -r -p "  start_yaw: " syaw
                read -r -p "  target_x: " a
                read -r -p "  target_y: " b
                launch_bug bug2 "${w:-bug_easy.world}" "${a:-3.0}" "${b:-0.0}" \
                                "${sx:-0.0}" "${sy:-0.0}" "${syaw:-0.0}"
                ;;
            0) return ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

menu_diagnostic() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}DIAGNÓSTICO ROS${C_RESET}\n"
        info "Estos comandos requieren que un launch esté corriendo en otra terminal"
        echo
        echo "  1)  Lista de nodos activos"
        echo "  2)  Lista de topics publicados"
        echo "  3)  Estado actual del Bug (/bug/state)"
        echo "  4)  Distancias del LiDAR por sector"
        echo "  5)  Generar árbol TF (PDF → output/)"
        echo "  6)  Grafo de nodos (rqt_graph)"
        echo "  7)  Abrir carpeta output/"
        echo "  0)  Volver"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) show_nodes ;;
            2) show_topics ;;
            3) show_state ;;
            4) show_distances ;;
            5) show_tf_tree ;;
            6) show_rqt_graph ;;
            7) open_output_dir ;;
            0) return ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

main_menu() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}MENÚ PRINCIPAL${C_RESET}\n"
        echo -e "  ${C_MAGENTA}1)${C_RESET}  ${C_BOLD}Task 1${C_RESET}  ·  Bug 0 (avoidance + Σ_k visible en RViz)"
        echo -e "  ${C_MAGENTA}2)${C_RESET}  ${C_BOLD}Task 2${C_RESET}  ·  Bug 2 (con tracking de m-line)"
        echo -e "  ${C_MAGENTA}3)${C_RESET}  Diagnóstico ROS  (nodos · topics · estado · TF)"
        echo
        echo -e "  ${C_DIM}── Mantenimiento ──${C_RESET}"
        echo -e "  ${C_MAGENTA}4)${C_RESET}  Recompilar el paquete"
        echo -e "  ${C_MAGENTA}5)${C_RESET}  Limpieza total (rm build install log) + rebuild"
        echo
        echo -e "  ${C_MAGENTA}0)${C_RESET}  Salir"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) menu_task1 ;;
            2) menu_task2 ;;
            3) menu_diagnostic ;;
            4) build_pkg; pause ;;
            5) clean_build; pause ;;
            0) echo -e "\n${C_GREEN}Hasta luego.${C_RESET}\n"; exit 0 ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

# ── Entry point ──────────────────────────────────────────────────────────────
banner
ensure_dirs
ensure_sourced
sleep 1
main_menu
