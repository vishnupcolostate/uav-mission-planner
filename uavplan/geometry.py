"""2-D computational geometry for no-fly-zone avoidance.

Polygons are lists of (x, y) vertices in meters, without the closing vertex
repeated. Every predicate here is exact except for a small epsilon used to
treat "touching" as "not crossing", which lets a path graze a buffered
corner without being rejected.
"""
from __future__ import annotations

import math
from typing import List, Sequence

from .geo import XY

Polygon = List[XY]
EPS = 1e-9


def dist(a: XY, b: XY) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def cross(o: XY, a: XY, b: XY) -> float:
    """z-component of (a - o) x (b - o): >0 if o->a->b turns left."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def signed_area(poly: Sequence[XY]) -> float:
    return 0.5 * sum(
        poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
        for i in range(len(poly))
    )


def ccw(poly: Sequence[XY]) -> Polygon:
    """Return the polygon with counter-clockwise winding."""
    return list(poly) if signed_area(poly) > 0 else list(reversed(poly))


def segments_cross(a: XY, b: XY, c: XY, d: XY) -> bool:
    """True if segments ab and cd properly intersect (cross at an interior point)."""
    d1, d2 = cross(c, d, a), cross(c, d, b)
    d3, d4 = cross(a, b, c), cross(a, b, d)
    return ((d1 > EPS and d2 < -EPS) or (d1 < -EPS and d2 > EPS)) and \
           ((d3 > EPS and d4 < -EPS) or (d3 < -EPS and d4 > EPS))


def point_in_polygon(p: XY, poly: Sequence[XY]) -> bool:
    """Ray casting: count edge crossings of a ray from p toward +x."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_at_y:
                inside = not inside
    return inside


def segment_hits_polygon(a: XY, b: XY, poly: Sequence[XY]) -> bool:
    """True if segment ab passes through the interior of poly.

    Either it properly crosses an edge, or it lies along a chord between two
    boundary points (no proper crossing, but its midpoint is inside).
    """
    n = len(poly)
    for i in range(n):
        if segments_cross(a, b, poly[i], poly[(i + 1) % n]):
            return True
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    return point_in_polygon(mid, poly) and not point_on_boundary(mid, poly)


def point_on_boundary(p: XY, poly: Sequence[XY], tol: float = 1e-7) -> bool:
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if abs(cross(a, b, p)) <= tol * max(dist(a, b), 1.0) and \
           min(a[0], b[0]) - tol <= p[0] <= max(a[0], b[0]) + tol and \
           min(a[1], b[1]) - tol <= p[1] <= max(a[1], b[1]) + tol:
            return True
    return False


def inflate(poly: Sequence[XY], d: float) -> Polygon:
    """Offset every edge outward by d and join with mitered corners.

    Each new vertex is where two neighbouring offset edges meet, so every
    edge of the result is exactly d from the matching original edge. Exact
    for convex polygons; for concave ones keep d small relative to features.
    """
    poly = ccw(poly)
    n = len(poly)
    out: Polygon = []
    for i in range(n):
        prev, cur, nxt = poly[i - 1], poly[i], poly[(i + 1) % n]
        # Outward normals of the two edges meeting at `cur` (CCW => outward is to the right).
        n1 = _unit_normal(prev, cur)
        n2 = _unit_normal(cur, nxt)
        bis = (n1[0] + n2[0], n1[1] + n2[1])
        cos_half = (1 + n1[0] * n2[0] + n1[1] * n2[1]) / 2
        scale = d / max(math.sqrt(max(cos_half, 0.0)), 1e-6) / max(math.hypot(*bis), 1e-12)
        out.append((cur[0] + bis[0] * scale, cur[1] + bis[1] * scale))
    return out


def _unit_normal(a: XY, b: XY) -> XY:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    return (dy / length, -dx / length)


def convex_vertices(poly: Sequence[XY]) -> List[XY]:
    """Vertices where the (CCW) boundary turns left.

    Shortest paths around obstacles only ever bend at convex corners, so these
    are the only vertices worth adding to the visibility graph.
    """
    poly = ccw(poly)
    n = len(poly)
    return [poly[i] for i in range(n) if cross(poly[i - 1], poly[i], poly[(i + 1) % n]) > EPS]


def polyline_length(points: Sequence[XY]) -> float:
    return sum(dist(points[i], points[i + 1]) for i in range(len(points) - 1))
