"""
occupancy_grid — Occupancy grid binaria construida desde rectángulos.

Implementación desde cero con SOLO numpy (sin scipy, sin OpenCV, sin
cualquier librería de imágenes). Cumple restricción dura del PDF MCR2.

CONVENCIÓN DE COORDENADAS:
    Mundo (Gazebo):   x, y en metros, origen del world
    Grid (numpy):     row=índice Y, col=índice X
                      grid[0,0]   = esquina (origin_x, origin_y)
                      grid[N-1,M-1] = esquina (origin_x + W, origin_y + H)
    Valor de celda:   0 = libre, 1 = ocupado

JUSTIFICACIÓN DE PARÁMETROS:
    resolution = 0.05 m/celda
        El Puzzlebot mide ~0.20m de ancho. Con celdas de 5cm tenemos
        4 celdas de robot → suficiente granularidad para planificación
        sin overhead de memoria.

    robot_radius = 0.15 m
        Radio físico del Puzzlebot + margen de seguridad. Se usa para
        inflar obstáculos (configuration-space planning): el robot
        se trata como un punto y los muros se "engordan" por robot_radius.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


class OccupancyGrid:
    """Occupancy grid 2D binaria con conversiones mundo↔celda e inflación.

    Attributes:
        width_m: ancho del mapa en metros.
        height_m: alto del mapa en metros.
        resolution: tamaño de celda en metros (típico 0.05).
        origin_x, origin_y: coordenadas del mundo de grid[0,0].
        rows, cols: dimensiones del numpy array (rows = Y, cols = X).
        grid: numpy array (rows, cols) con dtype uint8. 0=libre, 1=ocupado.
    """

    def __init__(
        self,
        width_m: float,
        height_m: float,
        resolution: float = 0.05,
        origin: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        if width_m <= 0 or height_m <= 0:
            raise ValueError(
                f'width_m and height_m must be positive, got '
                f'({width_m}, {height_m})')
        if resolution <= 0:
            raise ValueError(f'resolution must be positive, got {resolution}')

        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.resolution = float(resolution)
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])

        self.cols = int(math.ceil(width_m / resolution))
        self.rows = int(math.ceil(height_m / resolution))
        # uint8 para minimizar memoria. 1=ocupado, 0=libre.
        self.grid: np.ndarray = np.zeros((self.rows, self.cols), dtype=np.uint8)

    # ── Conversiones mundo ↔ grid ────────────────────────────────────────────

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convierte coordenadas mundo (m) a (row, col) del grid.

        Args:
            x, y: coordenadas en metros.

        Returns:
            (row, col) — row = índice en Y, col = índice en X.
            Si están fuera del grid, devuelve los más cercanos válidos
            (clipping). Usa is_inside_world() si quieres validar antes.
        """
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        # Clip a rango válido.
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convierte (row, col) del grid a centro de celda en coordenadas mundo.

        Args:
            row: índice de fila (eje Y).
            col: índice de columna (eje X).

        Returns:
            (x, y) en metros — CENTRO de la celda.
        """
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def is_inside_world(self, x: float, y: float) -> bool:
        """¿La coordenada mundo cae dentro del grid?"""
        return (self.origin_x <= x < self.origin_x + self.width_m
                and self.origin_y <= y < self.origin_y + self.height_m)

    # ── Definición de obstáculos ─────────────────────────────────────────────

    def add_rect_obstacle(
        self,
        cx: float,
        cy: float,
        w: float,
        h: float,
    ) -> None:
        """Marca como ocupado un rectángulo definido por su CENTRO y tamaño.

        Args:
            cx, cy: centro del rectángulo en coordenadas mundo (m).
            w: ancho (extensión total en X) en metros.
            h: alto (extensión total en Y) en metros.
        """
        # Convertimos las 4 esquinas a celdas y marcamos el bounding box.
        x_min = cx - w / 2.0
        x_max = cx + w / 2.0
        y_min = cy - h / 2.0
        y_max = cy + h / 2.0

        r_min, c_min = self.world_to_grid(x_min, y_min)
        r_max, c_max = self.world_to_grid(x_max, y_max)

        # Garantizar orden creciente.
        r_lo, r_hi = min(r_min, r_max), max(r_min, r_max)
        c_lo, c_hi = min(c_min, c_max), max(c_min, c_max)

        self.grid[r_lo:r_hi + 1, c_lo:c_hi + 1] = 1

    def add_wall_perimeter(self, thickness_cells: int = 1) -> None:
        """Agrega un perímetro ocupado a todo el borde del grid.

        Útil para garantizar que el A* nunca planee salir del mundo.
        thickness_cells suele ser 1 (1 celda = 5cm de muro virtual).
        """
        t = max(1, thickness_cells)
        self.grid[:t, :] = 1
        self.grid[-t:, :] = 1
        self.grid[:, :t] = 1
        self.grid[:, -t:] = 1

    # ── Inflación (configuration space) ──────────────────────────────────────

    def inflate(self, robot_radius: float) -> None:
        """Infla los obstáculos por robot_radius (en metros).

        Esto convierte el grid a configuration-space: el robot se trata
        como un punto y los obstáculos se "engordan" por su radio. Es la
        técnica estándar de planificación cuando el robot es circular.

        Implementación SIN scipy: doble for loop con máscara circular.
        Complejidad: O(rows * cols * radius_cells²). Aceptable para
        grids pequeños (~100x100). Para grids grandes habría que usar
        distance transform pero ese requiere scipy.

        Args:
            robot_radius: radio del robot en metros.
        """
        radius_cells = int(math.ceil(robot_radius / self.resolution))
        if radius_cells <= 0:
            return

        # Pre-calcular máscara circular: offsets (dr, dc) que caen dentro
        # del círculo de radio radius_cells.
        circle_offsets = []
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc <= radius_cells * radius_cells:
                    circle_offsets.append((dr, dc))

        # Encontrar celdas ocupadas originales.
        occupied = np.argwhere(self.grid == 1)

        # Crear nuevo grid e inflar.
        inflated = self.grid.copy()
        rows, cols = self.rows, self.cols
        for r, c in occupied:
            for dr, dc in circle_offsets:
                rr, cc = r + dr, c + dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    inflated[rr, cc] = 1

        self.grid = inflated

    # ── Consultas para A* ────────────────────────────────────────────────────

    def is_free(self, row: int, col: int) -> bool:
        """¿La celda (row, col) está libre y dentro del grid?"""
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return False
        return bool(self.grid[row, col] == 0)

    def is_free_world(self, x: float, y: float) -> bool:
        """¿La coordenada mundo (x, y) cae en celda libre?"""
        if not self.is_inside_world(x, y):
            return False
        r, c = self.world_to_grid(x, y)
        return self.is_free(r, c)

    # ── Visualización / debug ────────────────────────────────────────────────

    def to_numpy(self) -> np.ndarray:
        """Devuelve una copia del grid como numpy uint8."""
        return self.grid.copy()

    def to_ros_occupancy(self) -> np.ndarray:
        """Convierte a formato nav_msgs/OccupancyGrid: int8, 0=libre, 100=ocup."""
        # ROS espera int8 con 0 (libre), 100 (ocupado), -1 (desconocido).
        out = np.zeros_like(self.grid, dtype=np.int8)
        out[self.grid == 1] = 100
        return out

    def __repr__(self) -> str:
        n_occ = int(np.sum(self.grid))
        total = self.rows * self.cols
        return (f'OccupancyGrid({self.rows}x{self.cols} cells, '
                f'{self.resolution}m/cell, '
                f'origin=({self.origin_x}, {self.origin_y}), '
                f'occupied={n_occ}/{total} ({100*n_occ/total:.1f}%))')


# ── Helper para cargar mapa desde YAML ───────────────────────────────────────

def grid_from_yaml(yaml_data: dict) -> OccupancyGrid:
    """Construye un OccupancyGrid desde un dict cargado de YAML.

    Formato esperado:
        grid:
          width_m: 7.0
          height_m: 6.0
          resolution: 0.05
          origin: [-1.0, -1.0]
          robot_radius: 0.15
        obstacles:
          - {type: rect, x: 1.15, y: 3.5, w: 1.3, h: 1.6}
          - ...

    Args:
        yaml_data: dict con keys 'grid' y 'obstacles'.

    Returns:
        OccupancyGrid con obstáculos cargados e inflados.
    """
    g_cfg = yaml_data['grid']
    grid = OccupancyGrid(
        width_m=g_cfg['width_m'],
        height_m=g_cfg['height_m'],
        resolution=g_cfg.get('resolution', 0.05),
        origin=tuple(g_cfg.get('origin', [0.0, 0.0])),
    )

    for obs in yaml_data.get('obstacles', []):
        if obs['type'] == 'rect':
            grid.add_rect_obstacle(obs['x'], obs['y'], obs['w'], obs['h'])
        else:
            raise ValueError(f"Unknown obstacle type: {obs['type']}")

    # Perímetro virtual para no planificar fuera del mundo.
    if g_cfg.get('add_perimeter', True):
        grid.add_wall_perimeter(thickness_cells=1)

    # Inflación final con el radio del robot.
    robot_radius = g_cfg.get('robot_radius', 0.15)
    grid.inflate(robot_radius)

    return grid
