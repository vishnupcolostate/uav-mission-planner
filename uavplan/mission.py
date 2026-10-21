"""Mission file -> flight plan.

Pipeline:
  1. project home, waypoints, and no-fly zones into local meters
  2. build a visibility graph around the buffered no-fly zones
  3. compute obstacle-aware shortest paths between every pair of stops
  4. order the stops (NN -> 2-opt -> Or-opt) on that distance matrix
  5. split the tour into sorties that fit the battery range
  6. expand each leg back into its detour polyline and export GeoJSON
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

from .geo import LocalProjection
from .geometry import Polygon, polyline_length
from .pathfind import VisibilityGraph
from .sorties import clarke_wright, sortie_length, split_best_rotation
from .tour import nearest_neighbour, optimise, tour_length


@dataclass
class Waypoint:
    id: str
    lat: float
    lon: float


@dataclass
class NoFlyZone:
    name: str
    polygon: List[Tuple[float, float]]  # (lat, lon)


@dataclass
class Mission:
    name: str
    home: Waypoint
    waypoints: List[Waypoint]
    no_fly_zones: List[NoFlyZone]
    cruise_speed_mps: float
    max_range_m: float
    loiter_s: float = 0.0
    safety_buffer_m: float = 50.0

    @classmethod
    def load(cls, path: str | Path) -> "Mission":
        raw = json.loads(Path(path).read_text())
        uav = raw["uav"]
        return cls(
            name=raw["name"],
            home=Waypoint("HOME", raw["home"]["lat"], raw["home"]["lon"]),
            waypoints=[Waypoint(w["id"], w["lat"], w["lon"]) for w in raw["waypoints"]],
            no_fly_zones=[NoFlyZone(z["name"], [tuple(p) for p in z["polygon"]])
                          for z in raw.get("no_fly_zones", [])],
            cruise_speed_mps=uav["cruise_speed_mps"],
            max_range_m=uav["max_range_m"],
            loiter_s=uav.get("loiter_s", 0.0),
            safety_buffer_m=raw.get("safety_buffer_m", 50.0),
        )


@dataclass
class Plan:
    mission: Mission
    order: List[int]                      # indices into stops (0 = home)
    sorties: List[List[int]]
    legs: Dict[Tuple[int, int], List[Tuple[float, float]]]  # (i, j) -> lat/lon polyline
    matrix: List[List[float]]             # obstacle-aware distances, meters
    distance_m: float
    strategy: str                         # which sortie-building method won
    loop_distance_m: float                # optimised single loop, unlimited battery
    naive_distance_m: float               # nearest-neighbour loop, for comparison
    straight_line_m: float                # same sorties, if no-fly zones are ignored
    keep_out: List[List[Tuple[float, float]]] = field(default_factory=list)

    @property
    def stops(self) -> List[Waypoint]:
        return [self.mission.home] + self.mission.waypoints

    @property
    def flight_time_s(self) -> float:
        return self.distance_m / self.mission.cruise_speed_mps + self.mission.loiter_s * len(self.mission.waypoints)


def plan(m: Mission) -> Plan:
    proj = LocalProjection(m.home.lat, m.home.lon)
    stops = [m.home] + m.waypoints
    xy = [proj.to_xy(s.lat, s.lon) for s in stops]
    obstacles: List[Polygon] = [[proj.to_xy(lat, lon) for lat, lon in z.polygon] for z in m.no_fly_zones]
    graph = VisibilityGraph(obstacles, m.safety_buffer_m)

    n = len(stops)
    d = [[0.0] * n for _ in range(n)]
    paths: Dict[Tuple[int, int], List] = {}
    for i in range(n):
        for j in range(i + 1, n):
            path = graph.shortest_path(xy[i], xy[j])
            paths[(i, j)], paths[(j, i)] = path, path[::-1]
            d[i][j] = d[j][i] = polyline_length(path)

    order = optimise(d)
    # Two ways to fit the battery: cut the optimised loop into pieces, or build
    # sorties directly with Clarke-Wright. Polish each sortie, keep the shorter plan.
    candidates = {
        "split loop": split_best_rotation(order, d, m.max_range_m, labels=[s.id for s in stops]),
        "Clarke-Wright": clarke_wright(d, m.max_range_m),
    }
    strategy, sorties = min(
        ((name, [_polish(s, d) for s in c]) for name, c in candidates.items()),
        key=lambda item: sum(sortie_length(s, d) for s in item[1]),
    )
    straight = [[((xy[i][0] - xy[j][0]) ** 2 + (xy[i][1] - xy[j][1]) ** 2) ** 0.5 for j in range(n)] for i in range(n)]

    return Plan(
        mission=m,
        order=order,
        sorties=sorties,
        legs={k: [proj.to_latlon(x, y) for x, y in v] for k, v in paths.items()},
        matrix=d,
        distance_m=sum(sortie_length(s, d) for s in sorties),
        strategy=strategy,
        loop_distance_m=tour_length(order, d),
        naive_distance_m=tour_length(nearest_neighbour(d), d),
        straight_line_m=sum(sortie_length(s, straight) for s in sorties),
        keep_out=[[proj.to_latlon(x, y) for x, y in poly] for poly in graph.keep_out],
    )


def _polish(sortie: List[int], d) -> List[int]:
    """Re-optimise the stop order inside one sortie (home stays first and last)."""
    sub = sortie[:-1]
    if len(sub) <= 3:
        return sortie
    local = [[d[a][b] for b in sub] for a in sub]
    return [sub[i] for i in optimise(local)] + [sub[0]]


def to_geojson(p: Plan) -> dict:
    """GeoJSON uses [lon, lat] order."""
    m = p.mission
    features = []
    for z in m.no_fly_zones:
        ring = [[lon, lat] for lat, lon in z.polygon]
        features.append({"type": "Feature", "properties": {"kind": "nfz", "name": z.name},
                         "geometry": {"type": "Polygon", "coordinates": [ring + ring[:1]]}})
    for z, poly in zip(m.no_fly_zones, p.keep_out):
        ring = [[lon, lat] for lat, lon in poly]
        features.append({"type": "Feature", "properties": {"kind": "buffer", "name": z.name},
                         "geometry": {"type": "Polygon", "coordinates": [ring + ring[:1]]}})
    seq = {stop: k for k, stop in enumerate(p.order)}
    for i, s in enumerate(p.stops):
        features.append({"type": "Feature",
                         "properties": {"kind": "home" if i == 0 else "waypoint", "id": s.id, "seq": seq[i]},
                         "geometry": {"type": "Point", "coordinates": [s.lon, s.lat]}})
    for k, sortie in enumerate(p.sorties, start=1):
        coords: List[List[float]] = []
        for a, b in zip(sortie, sortie[1:]):
            leg = [[lon, lat] for lat, lon in p.legs[(a, b)]]
            coords.extend(leg if not coords else leg[1:])
        features.append({"type": "Feature",
                         "properties": {"kind": "sortie", "sortie": k,
                                        "stops": [p.stops[i].id for i in sortie]},
                         "geometry": {"type": "LineString", "coordinates": coords}})
    return {
        "type": "FeatureCollection",
        "properties": {
            "mission": m.name, "distance_m": round(p.distance_m), "flight_time_s": round(p.flight_time_s),
            "sorties": len(p.sorties), "strategy": p.strategy,
            "loop_distance_m": round(p.loop_distance_m), "naive_distance_m": round(p.naive_distance_m),
            "max_range_m": m.max_range_m, "cruise_speed_mps": m.cruise_speed_mps,
        },
        "features": features,
    }
