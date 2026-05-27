"""
_status_panel — Panel de status en terminal para Bug 0 / Bug 2 (Part 2).

Output periódico, multiline, con colores ANSI. Diseñado para que se lea en
una terminal típica de 80 columnas, una "tarjeta" cada ~1.5 s. Reporta:

    · Tiempo desde arranque
    · Algoritmo (Bug 0 / Bug 2)
    · Estado FSM coloreado (GO_TO_GOAL verde, WALL_FOLLOW amarillo, etc.)
    · Waypoint actual (idx/total) y coordenadas
    · Pose del robot (x, y, yaw°)
    · Distancia al goal, distancia recorrida bordeando
    · Distancias LiDAR (front/left/right) con barra ASCII
    · Updates / rejects del EKF (si lo expone el bug)

No depende de rich/curses — solo strings + ANSI escapes. Si la terminal
no soporta colores, se ve igual de bien (solo sin coloreado).
"""

from __future__ import annotations

import math


# ── ANSI ─────────────────────────────────────────────────────────────────────
RESET   = '\033[0m'
BOLD    = '\033[1m'
DIM     = '\033[2m'
RED     = '\033[31m'
GREEN   = '\033[32m'
YELLOW  = '\033[33m'
BLUE    = '\033[34m'
MAGENTA = '\033[35m'
CYAN    = '\033[36m'
WHITE   = '\033[37m'
GREY    = '\033[90m'


def _color_state(state: str) -> str:
    """Colorea el nombre del estado del Bug FSM."""
    if state == 'GO_TO_GOAL':
        return f'{GREEN}{BOLD}{state}{RESET}'
    if state == 'WALL_FOLLOW':
        return f'{YELLOW}{BOLD}{state}{RESET}'
    if state == 'GOAL_REACHED':
        return f'{CYAN}{BOLD}{state}{RESET}'
    return f'{WHITE}{state}{RESET}'


def _bar(value: float, max_value: float = 2.0, width: int = 16,
         danger: float = 0.5, warn: float = 1.0) -> str:
    """Devuelve una barra ASCII coloreada según el rango del valor.

    Roja si v < danger (peligro de choque),
    amarilla si v < warn,
    verde si v >= warn.
    inf → '████ ∞'.
    """
    if math.isinf(value):
        return f'{GREEN}{"█" * width}{RESET} {GREY}∞{RESET}'

    v = max(0.0, min(value, max_value))
    filled = int((v / max_value) * width)
    if value < danger:
        color = RED
    elif value < warn:
        color = YELLOW
    else:
        color = GREEN
    return f'{color}{"█" * filled}{GREY}{"░" * (width - filled)}{RESET}'


def render(*, algo: str, t_s: float, state: str,
           wp_idx: int, wp_total: int, wp_xy: tuple[float, float],
           pose: tuple[float, float, float],
           d_goal: float, d_front: float, d_left: float, d_right: float,
           extra_line: str | None = None) -> str:
    """Construye el panel multiline y lo devuelve como string."""
    x, y, yaw = pose
    yaw_deg = math.degrees(yaw)
    gx, gy = wp_xy

    title = f' {algo}  ·  Part 2 — Multi-waypoint Navigation '
    pad = max(0, 76 - len(title))
    head = f'{CYAN}{BOLD}┌─{title}{"─" * pad}─┐{RESET}'
    bot  = f'{CYAN}{BOLD}└{"─" * 78}┘{RESET}'

    def row(s: str) -> str:
        # quita ANSI escapes para medir el ancho real
        import re
        visible = re.sub(r'\033\[[0-9;]*m', '', s)
        pad = max(0, 76 - len(visible))
        return f'{CYAN}│{RESET} {s}{" " * pad} {CYAN}│{RESET}'

    lines = [head]
    lines.append(row(f'{DIM}t={t_s:6.1f}s{RESET}   '
                     f'state: {_color_state(state)}'))
    lines.append(row(f'{DIM}waypoint:{RESET} '
                     f'{BOLD}{wp_idx + 1}/{wp_total}{RESET}  '
                     f'→  ({BLUE}{gx:+.2f}{RESET}, {BLUE}{gy:+.2f}{RESET}) m'))
    lines.append(row(f'{DIM}pose:{RESET}     '
                     f'({MAGENTA}{x:+.2f}{RESET}, {MAGENTA}{y:+.2f}{RESET}) m   '
                     f'yaw={MAGENTA}{yaw_deg:+6.1f}°{RESET}   '
                     f'd_goal={BOLD}{d_goal:.2f}{RESET} m'))
    lines.append(row(f'{DIM}LiDAR:{RESET}'))
    lines.append(row(f'  front  {_bar(d_front)} {d_front:5.2f} m'))
    lines.append(row(f'  left   {_bar(d_left)} {d_left:5.2f} m'))
    lines.append(row(f'  right  {_bar(d_right)} {d_right:5.2f} m'))
    if extra_line:
        lines.append(row(extra_line))
    lines.append(bot)

    return '\n'.join(lines)
