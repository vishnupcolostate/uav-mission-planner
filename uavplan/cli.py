"""Command-line interface.

    python -m uavplan examples/fort_collins.json -o web/plan.geojson
"""
from __future__ import annotations

import argparse
import json
import time
from typing import List, Optional

from .mission import Mission, plan, to_geojson
from .sorties import sortie_length


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="uavplan", description="UAV mission planner")
    ap.add_argument("mission", help="mission JSON file")
    ap.add_argument("-o", "--out", default="web/plan.geojson", help="GeoJSON output path")
    args = ap.parse_args(argv)

    m = Mission.load(args.mission)
    t0 = time.perf_counter()
    p = plan(m)
    elapsed = time.perf_counter() - t0

    print(f"Mission: {m.name}")
    print(f"  {len(m.waypoints)} waypoints, {len(m.no_fly_zones)} no-fly zones "
          f"(+{m.safety_buffer_m:g} m buffer), planned in {elapsed * 1000:.0f} ms\n")
    for k, s in enumerate(p.sorties, start=1):
        km = sortie_length(s, p.matrix) / 1000
        print(f"  Sortie {k}: {' -> '.join(p.stops[i].id for i in s)}")
        print(f"            {km:.2f} km of {m.max_range_m / 1000:.1f} km range ({km * 1000 / m.max_range_m:.0%})")
    print()
    print(f"  Total distance       {p.distance_m / 1000:7.2f} km  ({len(p.sorties)} sorties, {p.strategy})")
    print(f"  Flight time          {p.flight_time_s / 60:7.1f} min at {m.cruise_speed_mps:g} m/s + "
          f"{m.loiter_s:g} s loiter per waypoint")
    print(f"  If NFZs ignored      {p.straight_line_m / 1000:7.2f} km  "
          f"(avoidance costs {p.distance_m / p.straight_line_m - 1:.1%})")
    print()
    print(f"  Single loop (no battery limit)")
    print(f"    nearest neighbour  {p.naive_distance_m / 1000:7.2f} km")
    print(f"    2-opt + Or-opt     {p.loop_distance_m / 1000:7.2f} km  "
          f"({1 - p.loop_distance_m / p.naive_distance_m:.1%} shorter)")

    with open(args.out, "w") as fh:
        json.dump(to_geojson(p), fh, indent=1)
    print(f"\nwrote {args.out}")
