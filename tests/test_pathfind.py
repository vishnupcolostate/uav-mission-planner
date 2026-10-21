import math

import pytest

from uavplan.geometry import polyline_length, segment_hits_polygon
from uavplan.pathfind import NODE_CLEARANCE_M, NoPathError, VisibilityGraph

SQUARE = [(-10, -10), (10, -10), (10, 10), (-10, 10)]


def test_unobstructed_path_is_straight():
    g = VisibilityGraph([SQUARE])
    assert g.shortest_path((-50, 50), (50, 50)) == [(-50, 50), (50, 50)]


def test_path_goes_around_and_is_optimal():
    g = VisibilityGraph([SQUARE])
    path = g.shortest_path((-30, 0), (30, 0))
    for a, b in zip(path, path[1:]):
        assert not segment_hits_polygon(a, b, SQUARE)
    # Optimal: start -> corner -> corner -> goal along one side of the square.
    c = 10 + NODE_CLEARANCE_M
    expected = 2 * math.hypot(30 - c, c) + 2 * c
    assert polyline_length(path) == pytest.approx(expected, rel=1e-6)


def test_buffer_pushes_path_out():
    tight = VisibilityGraph([SQUARE]).shortest_path((-30, 0), (30, 0))
    wide = VisibilityGraph([SQUARE], buffer_m=5).shortest_path((-30, 0), (30, 0))
    assert polyline_length(wide) > polyline_length(tight)
    assert all(abs(y) >= 15 for _, y in wide[1:-1])


def test_threads_gap_between_two_obstacles():
    left = [(-20, -50), (-2, -50), (-2, 50), (-20, 50)]
    right = [(2, -50), (20, -50), (20, 50), (2, 50)]
    path = VisibilityGraph([left, right]).shortest_path((0, -100), (0, 100))
    assert path == [(0, -100), (0, 100)]
    # A buffer that closes the gap forces the long way round.
    around = VisibilityGraph([left, right], buffer_m=3).shortest_path((0, -100), (0, 100))
    assert polyline_length(around) > 200


def test_start_inside_zone_rejected():
    with pytest.raises(NoPathError):
        VisibilityGraph([SQUARE]).shortest_path((0, 0), (50, 50))


def test_goal_enclosed_is_unreachable():
    ring = [  # four walls boxing in the origin with no gap
        [(-30, -30), (30, -30), (30, -25), (-30, -25)],
        [(-30, 25), (30, 25), (30, 30), (-30, 30)],
        [(-30, -30), (-25, -30), (-25, 30), (-30, 30)],
        [(25, -30), (30, -30), (30, 30), (25, 30)],
    ]
    with pytest.raises(NoPathError):
        VisibilityGraph(ring).shortest_path((0, 0), (100, 100))
