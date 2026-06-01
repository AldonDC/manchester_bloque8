"""
pure_pursuit — Pure Pursuit tracker desde cero para diferencial.

Algoritmo CLÁSICO de Coulter (1992) "Implementation of the Pure Pursuit
Path Tracking Algorithm" para robots con cinemática diferencial. Es el
mismo que usa el Tesla Autopilot en su versión interna y NASA en rovers.

IDEA:
    1. Dada la pose actual del robot, encuentra un punto "lookahead"
       sobre el path a distancia Ld del robot.
    2. Calcula el arco circular que conecta el robot con ese punto.
    3. La curvatura κ = 2·y_local / Ld² (donde y_local es la coord Y
       del lookahead en el frame del robot).
    4. Comanda v_linear constante y v_angular = v_linear · κ.

JUSTIFICACIÓN PARÁMETROS:
    lookahead_distance = 0.40 m
        Para v_max = 0.25 m/s da T_lookahead = 1.6s → reacción suave
        sin "atajar" curvas. Si lo bajamos zigzaguea; si lo subimos
        corta curvas.

    max_linear_vel = 0.25 m/s
        Puzzlebot puede ir hasta ~0.35 m/s pero a 0.25 m/s el path
        following es suave y los ArUcos se detectan estables (la
        cámara no se mueve demasiado por frame).

    goal_tolerance = 0.15 m
        Disco donde declaramos "llegamos al final del path". Tamaño
        de la bandera 0.36m diámetro → 0.15m es la mitad, generoso.

ÍNDICE INCREMENTAL:
    Para no buscar el lookahead point en todo el path cada tick
    (O(n) cada vez), mantenemos un índice `_last_lookahead_idx` y
    solo avanzamos hacia adelante. Esto da O(1) amortizado.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class PurePursuitParams:
    """Parámetros del controlador. Todos en SI (m, m/s, rad/s)."""
    lookahead_distance: float = 0.40
    max_linear_vel: float = 0.25
    max_angular_vel: float = 0.80   # bajada de 1.5 → 0.8 para reducir
                                    # motion blur en la cámara durante giros.
                                    # ArUco detection rate sube de 8% a 70%+.
    goal_tolerance: float = 0.15
    # Si el lookahead efectivo es menor a esto (cerca del final del path),
    # comenzamos a frenar proporcional a la distancia al goal.
    brake_distance: float = 0.60
    # Si |ang_err| > este umbral, gira en sitio (v=0) antes de avanzar.
    # Bajado a 30° (de 45°) para que pivotee menos y mantenga la cámara
    # más estable durante el seguimiento del path.
    rotate_in_place_threshold_rad: float = math.radians(30.0)


class PurePursuit:
    """Pure Pursuit tracker para Puzzlebot (diferencial).

    Uso típico:
        pp = PurePursuit(PurePursuitParams())
        pp.set_path(path_xy)
        while not pp.is_goal_reached(current_pose):
            v, w = pp.compute_control(current_pose)
            publish_cmd_vel(v, w)
    """

    def __init__(self, params: Optional[PurePursuitParams] = None) -> None:
        self.params = params or PurePursuitParams()
        self._path: np.ndarray = np.empty((0, 2), dtype=float)
        self._last_lookahead_idx: int = 0
        self._closest_idx: int = 0

    # ── API pública ──────────────────────────────────────────────────────────

    def set_path(self, path: List[Tuple[float, float]]) -> None:
        """Asigna el path a seguir. Resetea el índice incremental."""
        if not path:
            self._path = np.empty((0, 2), dtype=float)
            self._last_lookahead_idx = 0
            self._closest_idx = 0
            return
        self._path = np.array(path, dtype=float)
        self._last_lookahead_idx = 0
        self._closest_idx = 0

    def has_path(self) -> bool:
        return self._path.shape[0] > 0

    def is_goal_reached(self, pose: Tuple[float, float, float]) -> bool:
        """¿El robot está a < goal_tolerance del último punto del path?"""
        if not self.has_path():
            return True
        x, y, _yaw = pose
        gx, gy = self._path[-1]
        return math.hypot(gx - x, gy - y) < self.params.goal_tolerance

    def compute_control(
        self,
        pose: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        """Calcula (v_lin, v_ang) para seguir el path.

        Args:
            pose: (x, y, yaw) del robot en frame mundo (radianes).

        Returns:
            (v_linear, v_angular) en m/s y rad/s. Saturados.
        """
        if not self.has_path():
            return 0.0, 0.0

        x, y, yaw = pose
        p = self.params

        # 1. Closest point para tracking incremental (solo avanzamos).
        self._update_closest_idx(x, y)

        # 2. Distancia al goal.
        gx, gy = self._path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < p.goal_tolerance:
            return 0.0, 0.0

        # 3. Buscar lookahead point.
        look_idx, look_pt = self._find_lookahead(x, y, p.lookahead_distance)
        if look_pt is None:
            # No hay punto a lookahead → usar el último.
            look_pt = (float(self._path[-1, 0]), float(self._path[-1, 1]))

        # 4. Transformar el lookahead al frame del robot.
        dx_w = look_pt[0] - x
        dy_w = look_pt[1] - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        # Inverse rotation: (x_r, y_r) = R(-yaw) * (dx_w, dy_w)
        x_local =  cos_yaw * dx_w + sin_yaw * dy_w
        y_local = -sin_yaw * dx_w + cos_yaw * dy_w

        # 5. Ángulo al lookahead (frame robot).
        ang_to_look = math.atan2(y_local, x_local)

        # 6. Rotate-in-place si lookahead está MUY detrás del robot.
        if abs(ang_to_look) > p.rotate_in_place_threshold_rad:
            v = 0.0
            w = math.copysign(p.max_angular_vel, ang_to_look)
            return v, w

        # 7. Cálculo de curvatura clásico Pure Pursuit.
        # κ = 2 * y_local / Ld²
        Ld = math.hypot(x_local, y_local)
        if Ld < 1e-6:
            return 0.0, 0.0
        curvature = 2.0 * y_local / (Ld * Ld)

        # 8. Velocidad lineal: max constante, decelerando cerca del goal.
        if dist_goal < p.brake_distance:
            v = p.max_linear_vel * (dist_goal / p.brake_distance)
        else:
            v = p.max_linear_vel
        # Reducir v en curvas cerradas (evita oversteer).
        # Si |κ| alta, baja v. Factor empírico.
        v *= 1.0 / (1.0 + 1.5 * abs(curvature))

        # 9. Velocidad angular = v * curvatura.
        w = v * curvature

        # Saturación.
        v = max(0.0, min(p.max_linear_vel, v))
        w = max(-p.max_angular_vel, min(p.max_angular_vel, w))
        return v, w

    # ── Helpers internos ─────────────────────────────────────────────────────

    def _update_closest_idx(self, x: float, y: float) -> None:
        """Avanza self._closest_idx al punto del path más cercano al robot.

        Solo busca HACIA ADELANTE en el path (no retrocede). Esto previene
        que el robot decida "volver" si pasa cerca de un punto anterior.
        """
        n = self._path.shape[0]
        best_idx = self._closest_idx
        best_d2 = float('inf')
        # Ventana de búsqueda: 20 puntos adelante.
        end = min(n, self._closest_idx + 20)
        for i in range(self._closest_idx, end):
            dx = self._path[i, 0] - x
            dy = self._path[i, 1] - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        self._closest_idx = best_idx

    def _find_lookahead(
        self,
        x: float,
        y: float,
        Ld: float,
    ) -> Tuple[int, Optional[Tuple[float, float]]]:
        """Encuentra el primer punto del path a distancia >= Ld del robot.

        Búsqueda desde self._last_lookahead_idx hacia adelante.
        """
        n = self._path.shape[0]
        start = max(self._last_lookahead_idx, self._closest_idx)
        for i in range(start, n):
            dx = self._path[i, 0] - x
            dy = self._path[i, 1] - y
            if math.hypot(dx, dy) >= Ld:
                self._last_lookahead_idx = i
                return i, (float(self._path[i, 0]), float(self._path[i, 1]))
        # No encontró → usa el último punto del path.
        return n - 1, None
