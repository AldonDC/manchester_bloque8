"""
Known ArUco marker map.

The challenge models localisation against *known* fiducial markers: each
ArUco id has a fixed pose in the world. This module loads that mapping
from a YAML config and exposes simple lookup helpers.

YAML format (see config/aruco_map.yaml):

    markers:
      0: { x: 2.0, y: 0.0, yaw: 3.14 }
      1: { x: 0.0, y: 2.0, yaw: -1.57 }
      ...

Only x and y are used by the range+bearing model. yaw is kept for the
pose-based observation model (optional) and for RViz visualisation.
"""

from typing import Dict, Optional

import numpy as np
import yaml


class ArucoMap:
    """In-memory id -> pose lookup."""

    def __init__(self, markers: Dict[int, Dict[str, float]]):
        self._markers = {int(k): v for k, v in markers.items()}

    @classmethod
    def from_yaml(cls, path: str) -> "ArucoMap":
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        markers = data.get('markers', {})
        return cls(markers)

    def has(self, marker_id: int) -> bool:
        return int(marker_id) in self._markers

    def xy(self, marker_id: int) -> Optional[np.ndarray]:
        m = self._markers.get(int(marker_id))
        if m is None:
            return None
        return np.array([float(m['x']), float(m['y'])], dtype=float)

    def yaw(self, marker_id: int) -> Optional[float]:
        m = self._markers.get(int(marker_id))
        if m is None or 'yaw' not in m:
            return None
        return float(m['yaw'])

    def ids(self):
        return list(self._markers.keys())

    def __len__(self):
        return len(self._markers)

    def __repr__(self):
        return f'ArucoMap({len(self)} markers: {self.ids()})'
