"""
astar_planner — A* desde cero con heap binario (heapq stdlib).

Implementación completa según el algoritmo clásico de Hart, Nilsson &
Raphael (1968). Usa SOLO numpy + heapq (stdlib), sin librerías externas.

ALGORITMO A*:
    g(n) = costo desde inicio hasta n (real)
    h(n) = heurística desde n hasta goal (estimado)
    f(n) = g(n) + h(n)
    Expandir siempre el nodo con menor f.
    Si h es admisible (≤ costo real), A* es ÓPTIMO.

JUSTIFICACIÓN DE DISEÑO:
    8-conectividad (8 vecinos):
        Permite movimiento diagonal → trayectorias más suaves y cortas.
        Costo diagonal = sqrt(2) ≈ 1.414 (vs ortogonal=1.0). Sin esto
        el path quedaría escalonado con saltos solo NSEW.

    Heurística: distancia euclidiana
        h(n) = sqrt(dx² + dy²) con dx, dy en celdas.
        Admisible (siempre ≤ distancia real con 8-conectividad).
        Más informada que Manhattan para grids con diagonales.

    Heap binario (heapq) en lugar de fringe lineal:
        Operaciones O(log n) vs O(n). Para grids de ~100x100 da una
        mejora de ~100x en velocidad.

    closed set como numpy bool array:
        O(1) lookup vs O(log n) de un set Python. Crítico para grids
        densos donde re-visitamos muchas celdas.
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

from puzzlebot_nav.planning.occupancy_grid import OccupancyGrid


# 8-conectividad: (dr, dc, cost)
# Ortogonal cost=1.0, diagonal cost=sqrt(2)≈1.414
_NEIGHBORS_8 = (
    (-1,  0, 1.0),         # N
    ( 1,  0, 1.0),         # S
    ( 0, -1, 1.0),         # W
    ( 0,  1, 1.0),         # E
    (-1, -1, math.sqrt(2)),  # NW
    (-1,  1, math.sqrt(2)),  # NE
    ( 1, -1, math.sqrt(2)),  # SW
    ( 1,  1, math.sqrt(2)),  # SE
)


def _heuristic(r1: int, c1: int, r2: int, c2: int) -> float:
    """Distancia euclidiana en celdas. Admisible para 8-conectividad."""
    dr = r2 - r1
    dc = c2 - c1
    return math.sqrt(dr * dr + dc * dc)


def _reconstruct_path(
    came_from: dict,
    current: Tuple[int, int],
) -> List[Tuple[int, int]]:
    """Backtrack desde el goal hasta el start usando came_from dict."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(
    grid: OccupancyGrid,
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    diagonal: bool = True,
) -> Optional[List[Tuple[float, float]]]:
    """A* en el occupancy grid desde start a goal (coords mundo).

    Args:
        grid: instancia de OccupancyGrid.
        start_world: (x, y) en metros donde arranca el robot.
        goal_world: (x, y) en metros donde queremos llegar.
        diagonal: si True usa 8-conectividad (default), False usa 4.

    Returns:
        Lista de puntos (x, y) en coords mundo si encontró path.
        None si no hay path posible.
    """
    start = grid.world_to_grid(*start_world)
    goal = grid.world_to_grid(*goal_world)

    # Validación: si start o goal están en celda ocupada, intenta encontrar
    # la celda libre más cercana (típico cuando spawn está justo en pared
    # tras inflación).
    if not grid.is_free(*start):
        nearby = _nearest_free_cell(grid, start)
        if nearby is None:
            return None
        start = nearby
    if not grid.is_free(*goal):
        nearby = _nearest_free_cell(grid, goal)
        if nearby is None:
            return None
        goal = nearby

    rows, cols = grid.rows, grid.cols
    # Costos g acumulados. Inf inicial = "no visitado".
    g_score = np.full((rows, cols), np.inf, dtype=np.float64)
    g_score[start] = 0.0

    # Closed set: True si ya expandimos esta celda.
    closed = np.zeros((rows, cols), dtype=bool)

    # Open set: heap binario de (f_score, counter, (row, col))
    # counter rompe ties para que heapq no compare tuplas de celdas.
    counter = 0
    open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
    f_start = _heuristic(start[0], start[1], goal[0], goal[1])
    heapq.heappush(open_heap, (f_start, counter, start))

    came_from: dict = {}

    neighbors = _NEIGHBORS_8 if diagonal else _NEIGHBORS_8[:4]

    while open_heap:
        _f, _c, current = heapq.heappop(open_heap)
        if current == goal:
            cell_path = _reconstruct_path(came_from, current)
            # Convertir a coords mundo.
            return [grid.grid_to_world(r, c) for r, c in cell_path]

        if closed[current]:
            continue
        closed[current] = True

        cr, cc = current
        for dr, dc, cost in neighbors:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if closed[nr, nc]:
                continue
            if not grid.is_free(nr, nc):
                continue
            # Para diagonales, evitar atravesar esquinas de obstáculos.
            if dr != 0 and dc != 0:
                if not (grid.is_free(cr + dr, cc) and grid.is_free(cr, cc + dc)):
                    continue

            tentative_g = g_score[current] + cost
            if tentative_g < g_score[nr, nc]:
                came_from[(nr, nc)] = current
                g_score[nr, nc] = tentative_g
                f = tentative_g + _heuristic(nr, nc, goal[0], goal[1])
                counter += 1
                heapq.heappush(open_heap, (f, counter, (nr, nc)))

    return None  # no path


def _nearest_free_cell(
    grid: OccupancyGrid,
    cell: Tuple[int, int],
    max_radius: int = 20,
) -> Optional[Tuple[int, int]]:
    """BFS expandiéndose por anillos para encontrar la celda libre más cercana.

    Útil cuando start o goal cae en obstáculo (inflación demasiado agresiva
    o coordenadas ligeramente desalineadas).
    """
    if grid.is_free(*cell):
        return cell
    r0, c0 = cell
    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                # Solo el "anillo" actual: máx(|dr|,|dc|) == radius
                if max(abs(dr), abs(dc)) != radius:
                    continue
                rr, cc = r0 + dr, c0 + dc
                if grid.is_free(rr, cc):
                    return rr, cc
    return None


def plan_tour(
    grid: OccupancyGrid,
    waypoints: List[Tuple[float, float]],
    close_loop: bool = True,
) -> Optional[List[List[Tuple[float, float]]]]:
    """Planifica un tour visitando todos los waypoints en orden.

    Concatena A* entre cada par consecutivo. Si close_loop=True, añade
    también el segmento del último al primero.

    Args:
        grid: OccupancyGrid.
        waypoints: lista de (x, y) en coords mundo.
        close_loop: si True, cierra el tour volviendo a waypoints[0].

    Returns:
        Lista de segmentos. Cada segmento es una lista de puntos (x, y).
        None si algún segmento es inalcanzable.
    """
    if len(waypoints) < 2:
        return None

    segments = []
    n = len(waypoints)
    pairs = [(i, i + 1) for i in range(n - 1)]
    if close_loop:
        pairs.append((n - 1, 0))

    for i, j in pairs:
        seg = astar(grid, waypoints[i], waypoints[j])
        if seg is None:
            return None
        segments.append(seg)

    return segments


def concatenate_segments(
    segments: List[List[Tuple[float, float]]],
) -> List[Tuple[float, float]]:
    """Concatena segmentos en un solo path, evitando duplicar uniones."""
    if not segments:
        return []
    path = list(segments[0])
    for seg in segments[1:]:
        # El primer punto del siguiente segmento es el último del anterior;
        # lo saltamos para no duplicar.
        path.extend(seg[1:])
    return path
