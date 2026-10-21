"""Lat/lon <-> local meters.

All planning happens in a flat local frame (x = east, y = north, meters)
centred on the home position. An equirectangular projection is accurate to
well under 0.1% across the ~20 km a small UAV covers, which keeps the
geometry code simple and exact (straight lines stay straight).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

EARTH_RADIUS_M = 6_371_008.8

XY = Tuple[float, float]


@dataclass(frozen=True)
class LocalProjection:
    lat0: float
    lon0: float

    @property
    def _kx(self) -> float:
        return math.radians(1) * EARTH_RADIUS_M * math.cos(math.radians(self.lat0))

    @property
    def _ky(self) -> float:
        return math.radians(1) * EARTH_RADIUS_M

    def to_xy(self, lat: float, lon: float) -> XY:
        return ((lon - self.lon0) * self._kx, (lat - self.lat0) * self._ky)

    def to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        return (self.lat0 + y / self._ky, self.lon0 + x / self._kx)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))
