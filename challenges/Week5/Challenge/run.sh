#!/usr/bin/env bash
# =============================================================================
#  Mini Challenge 4 — Launcher interactivo
#  Manchester Robotics · Week 5
# =============================================================================

# Nota: NO usamos `set -u` porque los scripts de ROS Humble
# (/opt/ros/humble/setup.bash) referencian variables no definidas
# internamente y abortarían el script.

# ── Configuración ────────────────────────────────────────────────────────────
WORKSPACE="/home/alfonso/Documents/8 Semestre/manchester_bloque"
PKG="puzzlebot_sim_w5"
PKG_SRC="${WORKSPACE}/challenges/Week5/Challenge/puzzlebot_sim"
OUTPUT_DIR="${WORKSPACE}/challenges/Week5/Challenge/output"

# ── Colores ──────────────────────────────────────────────────────────────────
C_RESET='\033[0m'
C_BOLD='\033[1m'
C_DIM='\033[2m'
C_RED='\033[0;31m'
C_GREEN='\033[0;32m'
C_YELLOW='\033[0;33m'
C_BLUE='\033[0;34m'
C_MAGENTA='\033[0;35m'
C_CYAN='\033[0;36m'
C_WHITE='\033[1;37m'

# ── Helpers ──────────────────────────────────────────────────────────────────
banner() {
    clear
    echo -e "${C_CYAN}${C_BOLD}"
    cat << 'EOF'
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│    MINI CHALLENGE 4  ·  Propagación de Covarianza                        │
│                                                                          │
│    Week 5 · Manchester Robotics × NVIDIA                                 │
│    Autor: Alfonso Diaz                                                   │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
EOF
    echo -e "${C_RESET}"
}

section() {
    echo -e "\n${C_BLUE}${C_BOLD}── $1 ──${C_RESET}\n"
}

ok()   { echo -e "${C_GREEN}[ok]${C_RESET}    $1"; }
warn() { echo -e "${C_YELLOW}[warn]${C_RESET}  $1"; }
err()  { echo -e "${C_RED}[err]${C_RESET}   $1"; }
info() { echo -e "${C_CYAN}[info]${C_RESET}  $1"; }

ensure_dirs() {
    mkdir -p "$OUTPUT_DIR"
}

ensure_sourced() {
    cd "$WORKSPACE" || { err "No se puede acceder al workspace"; exit 1; }
    # shellcheck disable=SC1091
    { source /opt/ros/humble/setup.bash; } 2>/dev/null || true
    ok "ROS Humble sourceado"
    if [[ -f "install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        { source install/setup.bash; } 2>/dev/null || true
        ok "Workspace local sourceado"
    else
        warn "No existe install/. Compilando primero..."
        build_pkg
    fi
}

build_pkg() {
    section "Compilando ${PKG}"
    cd "$WORKSPACE" || exit 1
    colcon build --packages-select "$PKG" --paths "$PKG_SRC"
    # shellcheck disable=SC1091
    source install/setup.bash
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

launch_task1() {
    local shape="$1" kr="$2" kl="$3"
    section "TASK 1  ·  Trayectoria '${shape}'  ·  k_r=${kr}  k_l=${kl}"
    info "Ctrl+C para detener"
    sleep 1
    ros2 launch "$PKG" puzzlebot_challenge5_launch.py \
        shape:="$shape" k_r:="$kr" k_l:="$kl"
}

launch_task2_straight() {
    local kr="$1" kl="$2"
    section "TASK 2  ·  Recta 1 m  ·  k_r=${kr}  k_l=${kl}"
    info "Avance open-loop 6.67 s a 0.15 m/s"
    sleep 1
    ros2 launch "$PKG" experiment_straight_launch.py \
        k_r:="$kr" k_l:="$kl"
}

launch_task2_rotation() {
    local kr="$1" kl="$2"
    section "TASK 2  ·  Rotación en sitio  ·  k_r=${kr}  k_l=${kl}"
    info "Giro open-loop  ω = π/2 rad/s  durante 4 s"
    sleep 1
    ros2 launch "$PKG" experiment_rotation_launch.py \
        k_r:="$kr" k_l:="$kl"
}

# ── Diagnóstico ROS ──────────────────────────────────────────────────────────

show_nodes() {
    section "Nodos activos"
    ros2 node list
    pause
}

show_topics() {
    section "Topics publicados"
    ros2 topic list -t
    pause
}

show_tf_tree() {
    section "Árbol de Transformadas (TF tree)"
    ensure_dirs
    local stamp
    stamp="$(date +%Y%m%d_%H%M%S)"
    local out_pdf="${OUTPUT_DIR}/tf_tree_${stamp}.pdf"
    local out_gv="${OUTPUT_DIR}/tf_tree_${stamp}.gv"

    info "Generando árbol TF (5 s de captura)..."
    info "Asegúrate de que un launch esté corriendo en otra terminal."

    # view_frames genera frames.pdf y frames.gv en el cwd
    local tmpdir
    tmpdir="$(mktemp -d)"
    pushd "$tmpdir" > /dev/null || return 1
    timeout 8 ros2 run tf2_tools view_frames 2>&1 | tail -3
    popd > /dev/null || return 1

    # tf2_tools nombra el archivo como frames_YYYY-MM-DD_HH.MM.SS.pdf en versiones recientes
    local generated_pdf
    generated_pdf="$(ls -t "$tmpdir"/frames*.pdf 2>/dev/null | head -1)"
    local generated_gv
    generated_gv="$(ls -t "$tmpdir"/frames*.gv 2>/dev/null | head -1)"

    if [[ -n "$generated_pdf" && -f "$generated_pdf" ]]; then
        mv "$generated_pdf" "$out_pdf"
        [[ -n "$generated_gv" && -f "$generated_gv" ]] && mv "$generated_gv" "$out_gv"
        ok "PDF guardado en:"
        echo "       $out_pdf"
        [[ -f "$out_gv" ]] && echo "       $out_gv"
        if command -v xdg-open >/dev/null; then
            info "Abriendo PDF..."
            xdg-open "$out_pdf" 2>/dev/null &
        fi
    else
        err "No se generó. Verifica que el launch esté activo."
    fi
    rm -rf "$tmpdir"
    pause
}

show_rqt_graph() {
    section "Grafo de nodos (rqt_graph)"
    info "Abriendo rqt_graph en ventana separada..."
    info "Tip: en el dropdown elige 'Nodes/Topics (all)' para ver todo."
    rqt_graph &
    pause
}

inspect_odom() {
    section "Covarianza del topic /odom"
    info "Imprimiendo 1 mensaje. Ctrl+C si se queda esperando."
    ros2 topic echo /odom --once --field pose.covariance
    pause
}

show_topic_hz() {
    section "Frecuencias de topics principales"
    echo -e "${C_DIM}(5 segundos por topic)${C_RESET}"
    for t in /odom /cmd_vel /wr /wl /joint_states /tf; do
        echo -e "\n${C_YELLOW}── ${t} ──${C_RESET}"
        timeout 5 ros2 topic hz "$t" 2>&1 | tail -5
    done
    pause
}

show_architecture() {
    section "Arquitectura del sistema"
    cat << 'EOF'

    ┌─────────────────────┐   /set_point   ┌────────────┐
    │ Trajectory Set Point│ ─────────────► │ Controller │
    │     Generator       │                └─────┬──────┘
    └─────────────────────┘                      │ /cmd_vel
                                                 ▼
                                          ┌──────────────┐
                                          │  Real / Sim  │
                                          │     Robot    │
                                          └──────┬───────┘
                                                 │ /wr  /wl
                                                 ▼
                                          ┌──────────────┐
                                          │ Localisation │   ●  Σₖ (3x3)
                                          │ (Dead Reck.) │
                                          └──────┬───────┘
                                                 │ /odom
                          ┌──────────────────────┼──────────┐
                          ▼                      ▼          ▼
                  ┌──────────────┐  ┌────────────────┐  ┌──────┐
                  │coord_transf  │  │ joint_state_pub│  │ RVIZ │
                  └──────┬───────┘  └────────┬───────┘  └──────┘
                         │ /tf              │ /joint_states
                         └──────────────────┘
EOF
    pause
}

open_output_dir() {
    section "Carpeta de salida"
    ensure_dirs
    echo "  ${OUTPUT_DIR}"
    echo
    if [[ -n "$(ls "$OUTPUT_DIR" 2>/dev/null)" ]]; then
        ls -lh "$OUTPUT_DIR"
    else
        info "(carpeta vacía — aún no has generado nada)"
    fi
    echo
    if command -v xdg-open >/dev/null; then
        xdg-open "$OUTPUT_DIR" 2>/dev/null &
        ok "Abriendo en el explorador de archivos..."
    fi
    pause
}

# ── Menús ────────────────────────────────────────────────────────────────────

menu_task1() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}TASK 1  ·  Demostración con trayectoria${C_RESET}\n"
        echo "  1)  Cuadrado    (square)    k_r = k_l = 0.05   [default]"
        echo "  2)  Triángulo   (triangle)  k_r = k_l = 0.05"
        echo "  3)  Hexágono    (hexagon)   k_r = k_l = 0.05"
        echo "  4)  Cuadrado    ·  ruido BAJO   (k = 0.02)"
        echo "  5)  Cuadrado    ·  ruido ALTO   (k = 0.15)"
        echo "  6)  Personalizado (figura + k_r + k_l)"
        echo "  0)  Volver"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) launch_task1 square   0.05 0.05 ;;
            2) launch_task1 triangle 0.05 0.05 ;;
            3) launch_task1 hexagon  0.05 0.05 ;;
            4) launch_task1 square   0.02 0.02 ;;
            5) launch_task1 square   0.15 0.15 ;;
            6)
                read -r -p "  Figura [square/triangle/hexagon]: " s
                read -r -p "  k_r: " a
                read -r -p "  k_l: " b
                launch_task1 "${s:-square}" "${a:-0.05}" "${b:-0.05}"
                ;;
            0) return ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

menu_task2() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}TASK 2  ·  Experimentos de calibración${C_RESET}\n"
        echo -e "  ${C_DIM}── Recta 1 m (open-loop) ──${C_RESET}"
        echo "  1)  k_r = k_l = 0.05    [default]"
        echo "  2)  k_r = k_l = 0.02    (bajo)"
        echo "  3)  k_r = k_l = 0.10    (medio)"
        echo "  4)  k_r = k_l = 0.20    (alto)"
        echo
        echo -e "  ${C_DIM}── Rotación en sitio (open-loop) ──${C_RESET}"
        echo "  5)  k_r = k_l = 0.05    [default]"
        echo "  6)  k_r = k_l = 0.10    (medio)"
        echo "  7)  k_r = k_l = 0.20    (alto)"
        echo
        echo "  8)  Personalizado (tipo + k_r + k_l)"
        echo "  0)  Volver"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) launch_task2_straight 0.05 0.05 ;;
            2) launch_task2_straight 0.02 0.02 ;;
            3) launch_task2_straight 0.10 0.10 ;;
            4) launch_task2_straight 0.20 0.20 ;;
            5) launch_task2_rotation 0.05 0.05 ;;
            6) launch_task2_rotation 0.10 0.10 ;;
            7) launch_task2_rotation 0.20 0.20 ;;
            8)
                read -r -p "  Tipo [straight/rotation]: " t
                read -r -p "  k_r: " a
                read -r -p "  k_l: " b
                if [[ "${t:-straight}" == "rotation" ]]; then
                    launch_task2_rotation "${a:-0.05}" "${b:-0.05}"
                else
                    launch_task2_straight "${a:-0.05}" "${b:-0.05}"
                fi
                ;;
            0) return ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

menu_diagnostic() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}DIAGNÓSTICO ROS  ·  Inspeccionar el sistema${C_RESET}\n"
        info "Estos comandos requieren que un launch esté corriendo en otra terminal"
        echo
        echo "  1)  ●  Lista de nodos activos"
        echo "  2)  ●  Lista de topics publicados"
        echo "  3)  ●  Generar árbol TF  →  guarda PDF en  output/"
        echo "  4)  ●  Grafo de nodos (rqt_graph) en ventana"
        echo "  5)  ●  Inspeccionar covarianza de /odom"
        echo "  6)  ●  Frecuencias (Hz) de topics principales"
        echo "  7)  ●  Diagrama de arquitectura (ASCII)"
        echo "  8)  ●  Abrir carpeta  output/"
        echo "  0)  Volver"
        echo
        read -r -p "$(echo -e ${C_CYAN}▸ Elige opción: ${C_RESET})" opt
        case "$opt" in
            1) show_nodes ;;
            2) show_topics ;;
            3) show_tf_tree ;;
            4) show_rqt_graph ;;
            5) inspect_odom ;;
            6) show_topic_hz ;;
            7) show_architecture ;;
            8) open_output_dir ;;
            0) return ;;
            *) err "Opción inválida"; sleep 1 ;;
        esac
    done
}

main_menu() {
    while true; do
        banner
        echo -e "${C_WHITE}${C_BOLD}MENÚ PRINCIPAL${C_RESET}\n"
        echo -e "  ${C_MAGENTA}1)${C_RESET}  ${C_BOLD}Task 1${C_RESET}  ·  Demostración con trayectoria + elipses"
        echo -e "  ${C_MAGENTA}2)${C_RESET}  ${C_BOLD}Task 2${C_RESET}  ·  Calibrar k_r, k_l (experimentos)"
        echo -e "  ${C_MAGENTA}3)${C_RESET}  Diagnóstico ROS  (nodos · topics · TF · rqt_graph)"
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
