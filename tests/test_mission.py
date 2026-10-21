from pathlib import Path

import pytest

from uavplan.geo import LocalProjection, haversine_m
from uavplan.geometry import segment_hits_polygon
from uavplan.mission import Mission, plan, to_geojson
from uavplan.sorties import sortie_length

EXAMPLE = Path(__file__).resolve().parent.parent / "examples" / "fort_collins.json"


@pytest.fixture(scope="module")
def result():
    return plan(Mission.load(EXAMPLE))


def test_projection_matches_haversine():
    proj = LocalProjection(40.5765, -105.0810)
    x, y = proj.to_xy(40.5130, -105.0450)
    assert (x * x + y * y) ** 0.5 == pytest.approx(haversine_m(40.5765, -105.0810, 40.5130, -105.0450), rel=1e-3)
    assert proj.to_latlon(x, y) == pytest.approx((40.5130, -105.0450))


def test_no_flown_leg_enters_a_no_fly_zone(result):
    m = result.mission
    proj = LocalProjection(m.home.lat, m.home.lon)
    zones = [[proj.to_xy(*v) for v in z.polygon] for z in m.no_fly_zones]
    for sortie in result.sorties:
        for a, b in zip(sortie, sortie[1:]):
            leg = [proj.to_xy(*p) for p in result.legs[(a, b)]]
            for p, q in zip(leg, leg[1:]):
                assert not any(segment_hits_polygon(p, q, z) for z in zones)


def test_plan_respects_battery_and_visits_everything(result):
    for s in result.sorties:
        assert sortie_length(s, result.matrix) <= result.mission.max_range_m
    visited = sorted(i for s in result.sorties for i in s[1:-1])
    assert visited == list(range(1, len(result.stops)))


def test_optimiser_beats_nearest_neighbour(result):
    assert result.loop_distance_m < result.naive_distance_m


def test_geojson_shape(result):
    gj = to_geojson(result)
    kinds = [f["properties"]["kind"] for f in gj["features"]]
    assert kinds.count("waypoint") == 12 and kinds.count("home") == 1
    assert kinds.count("nfz") == kinds.count("buffer") == 4
    assert kinds.count("sortie") == gj["properties"]["sorties"] == len(result.sorties)
    for f in gj["features"]:
        if f["geometry"]["type"] == "Polygon":
            ring = f["geometry"]["coordinates"][0]
            assert ring[0] == ring[-1]                           # closed ring
            assert -106 < ring[0][0] < -104 and 40 < ring[0][1] < 41   # [lon, lat] order
