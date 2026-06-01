"""
path_smoother — Suaviza el path crudo de A* (escalonado por celdas).

El path de A* sale como una serie de centros de celda → trayectoria
escalonada con ángulos rectos de 45°/90°. Para que el Puzzlebot la
siga suavemente con Pure Pursuit hay que:

  1. Quitar puntos colineales (prune): no aportan información.
  2. Interpolar con curvas suaves (Catmull-Rom spline).

CATMULL-ROM SPLINE (implementado desde cero con numpy):
    Para 4 puntos consecutivos P0, P1, P2, P3 la curva entre P1 y P2 es:

        f(t) = 0.5 * ( (2*P1) +
                       (-P0 + P2) * t +
                       (2*P0 - 5*P1 + 4*P2 - P3) * t² +
                       (-P0 + 3*P1 - 3*P2 + P3) * t³ )

    para t ∈ [0, 1]. Pasa EXACTAMENTE por P1 y P2 (interpolante, no
    aproximante como Bezier). Continuidad C¹ (tangentes continuas).

    Ventaja vs Bezier: pasa por todos los puntos de control, no
    "se desvía" del path original.

JUSTIFICACIÓN n_points = 200 default:
    Path típico tras A* en grid 7x6m: ~30-50 puntos. Interpolar a 200
    da espaciado ~3cm entre puntos consecutivos → buen muestreo para
    Pure Pursuit con lookahead 0.4m (>10 puntos en el lookahead).
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


def prune_collinear(
    path: List[Tuple[float, float]],
    angle_tolerance_rad: float = 0.05,
) -> List[Tuple[float, float]]:
    """Elimina puntos intermedios colineales con sus vecinos.

    Para cada terceto consecutivo (A, B, C), si el ángulo en B es ~180°
    (segmento casi recto), elimina B.

    Args:
        path: lista de (x, y).
        angle_tolerance_rad: ángulo bajo el cual consideramos "colineal".
            Default 0.05 rad ≈ 2.86°. Estricto pero seguro.

    Returns:
        Path pruneado. Siempre conserva primer y último punto.
    """
    if len(path) <= 2:
        return list(path)

    pruned = [path[0]]
    for i in range(1, len(path) - 1):
        a = np.array(pruned[-1], dtype=float)
        b = np.array(path[i], dtype=float)
        c = np.array(path[i + 1], dtype=float)
        v1 = b - a
        v2 = c - b
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue  # punto duplicado, salta
        cos_angle = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
        # angle 0 = colineal (mismo sentido), π = ida-vuelta.
        angle = float(np.arccos(cos_angle))
        if angle > angle_tolerance_rad:
            pruned.append(path[i])
    pruned.append(path[-1])
    return pruned


def catmull_rom_segment(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    n: int = 20,
) -> np.ndarray:
    """Genera n puntos de la curva Catmull-Rom entre P1 y P2.

    Args:
        p0, p1, p2, p3: numpy arrays shape (2,).
        n: número de puntos a generar entre P1 y P2 (inclusivo P1, exclusivo P2).

    Returns:
        numpy array shape (n, 2) con los puntos interpolados.
    """
    t = np.linspace(0.0, 1.0, n, endpoint=False).reshape(-1, 1)
    t2 = t * t
    t3 = t2 * t
    # Fórmula clásica Catmull-Rom (uniform, no centripetal).
    out = 0.5 * (
        (2.0 * p1) +
        (-p0 + p2) * t +
        (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
        (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    )
    return out


def catmull_rom_path(
    control_points: List[Tuple[float, float]],
    points_per_segment: int = 20,
) -> List[Tuple[float, float]]:
    """Construye una curva Catmull-Rom completa interpolando todos los segmentos.

    Para los extremos (primer y último segmento) duplicamos el primer/último
    punto como "phantom" para tener 4 puntos. Esto es la práctica estándar.

    Args:
        control_points: lista de puntos por los que la curva DEBE pasar.
        points_per_segment: muestras entre cada par de control points.

    Returns:
        Lista (x, y) de la curva suavizada.
    """
    if len(control_points) < 2:
        return list(control_points)
    if len(control_points) == 2:
        # Sin curvatura posible, devolvemos segmento recto muestreado.
        p0 = np.array(control_points[0], dtype=float)
        p1 = np.array(control_points[1], dtype=float)
        t = np.linspace(0.0, 1.0, points_per_segment + 1).reshape(-1, 1)
        out = (1 - t) * p0 + t * p1
        return [tuple(p) for p in out]

    pts = [np.array(p, dtype=float) for p in control_points]
    # Phantom points en los extremos: extrapolación lineal.
    pts.insert(0, pts[0] + (pts[0] - pts[1]))
    pts.append(pts[-1] + (pts[-1] - pts[-2]))

    out_segments = []
    for i in range(len(pts) - 3):
        seg = catmull_rom_segment(
            pts[i], pts[i + 1], pts[i + 2], pts[i + 3],
            n=points_per_segment,
        )
        out_segments.append(seg)
    # Añadir el último punto exacto para cerrar.
    out_segments.append(np.array([control_points[-1]], dtype=float))

    arr = np.vstack(out_segments)
    return [(float(x), float(y)) for x, y in arr]


def smooth(
    path: List[Tuple[float, float]],
    angle_tolerance_rad: float = 0.05,
    points_per_segment: int = 20,
) -> List[Tuple[float, float]]:
    """Pipeline completo: prune + Catmull-Rom.

    Args:
        path: lista cruda de A*.
        angle_tolerance_rad: para prune_collinear.
        points_per_segment: densidad de la curva final.

    Returns:
        Path suavizado listo para Pure Pursuit.
    """
    pruned = prune_collinear(path, angle_tolerance_rad=angle_tolerance_rad)
    return catmull_rom_path(pruned, points_per_segment=points_per_segment)
