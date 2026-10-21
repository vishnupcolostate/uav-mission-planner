import math

import pytest

from uavplan import geometry as g

SQUARE = [(0, 0), (10, 0), (10, 10), (0, 10)]


def test_segments_cross():
    assert g.segments_cross((0, 0), (10, 10), (0, 10), (10, 0))
    assert not g.segments_cross((0, 0), (5, 5), (6, 6), (10, 10))     # collinear, disjoint
    assert not g.segments_cross((0, 0), (5, 0), (5, 0), (5, 5))       # touching at endpoint


def test_point_in_polygon():
    assert g.point_in_polygon((5, 5), SQUARE)
    assert not g.point_in_polygon((15, 5), SQUARE)
    # Concave "C" shape: the notch is outside.
    c_shape = [(0, 0), (10, 0), (10, 3), (3, 3), (3, 7), (10, 7), (10, 10), (0, 10)]
    assert g.point_in_polygon((1, 5), c_shape)
    assert not g.point_in_polygon((6, 5), c_shape)


@pytest.mark.parametrize("a,b,hits", [
    ((-5, 5), (15, 5), True),     # straight through
    ((-5, -5), (15, -5), False),  # passes below
    ((0, 0), (10, 10), True),     # diagonal chord between two corners
    ((0, 0), (10, 0), False),     # runs along an edge
    ((-5, 0), (0, 5), False),     # misses the corner
])
def test_segment_hits_polygon(a, b, hits):
    assert g.segment_hits_polygon(a, b, SQUARE) is hits


def test_inflate_square_is_mitred():
    big = sorted(g.inflate(SQUARE, 2))
    for got, want in zip(big, sorted([(-2, -2), (12, -2), (12, 12), (-2, 12)])):
        assert got == pytest.approx(want)


def test_inflate_keeps_edges_at_distance():
    tri = [(0, 0), (10, 0), (3, 8)]
    big = g.inflate(tri, 1.5)
    for i in range(3):
        # Each offset edge is parallel to and exactly 1.5 from the original.
        (ax, ay), (bx, by) = tri[i], tri[(i + 1) % 3]
        px, py = big[i]
        d = abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / math.hypot(bx - ax, by - ay)
        assert d == pytest.approx(1.5)


def test_inflate_handles_clockwise_input():
    for a, b in zip(sorted(g.inflate(SQUARE[::-1], 1)), sorted(g.inflate(SQUARE, 1))):
        assert a == pytest.approx(b)


def test_convex_vertices_skip_reflex_corner():
    l_shape = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]
    assert (4, 4) not in g.convex_vertices(l_shape)
    assert len(g.convex_vertices(l_shape)) == 5
