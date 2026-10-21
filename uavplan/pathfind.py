"""Shortest obstacle-avoiding paths with a visibility graph and A*.

Key fact: the shortest path between two points that avoids polygonal
obstacles is a polyline whose interior bends sit on obstacle corners. So
instead of discretising the map into a grid (which gives jagged, longer
paths), we build a graph whose nodes are obstacle corners and whose edges
are straight segments that don't pass through any obstacle, then run A*.
"""
from __future__ import annotations

import heapq
from typing import Dict, List, Sequence, Tuple

from .geo import XY
from .geometry import Polygon, convex_vertices, dist, inflate, point_in_polygon, segment_hits_polygon

# Graph nodes sit this much further out than the keep-out boundary, so edges
# that hug a corner are never mistaken for edges that cut through it.
NODE_CLEARANCE_M = 0.5


class NoPathError(RuntimeError):
    pass


class VisibilityGraph:
    def __init__(self, obstacles: Sequence[Polygon], buffer_m: float = 0.0):
        self.keep_out: List[Polygon] = [inflate(p, buffer_m) if buffer_m > 0 else list(p) for p in obstacles]
        corners = [inflate(p, buffer_m + NODE_CLEARANCE_M) for p in obstacles]
        self.nodes: List[XY] = [
            v for poly in corners for v in convex_vertices(poly) if self.is_free(v)
        ]
        # Precompute corner-to-corner visibility once; queries only add 2 nodes.
        self.adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(self.nodes))}
        for i in range(len(self.nodes)):
            for j in range(i + 1, len(self.nodes)):
                if self.visible(self.nodes[i], self.nodes[j]):
                    w = dist(self.nodes[i], self.nodes[j])
                    self.adj[i].append((j, w))
                    self.adj[j].append((i, w))

    def is_free(self, p: XY) -> bool:
        return not any(point_in_polygon(p, poly) for poly in self.keep_out)

    def visible(self, a: XY, b: XY) -> bool:
        return not any(segment_hits_polygon(a, b, poly) for poly in self.keep_out)

    def shortest_path(self, start: XY, goal: XY) -> List[XY]:
        """A* from start to goal; returns the polyline including both endpoints."""
        for label, p in (("start", start), ("goal", goal)):
            if not self.is_free(p):
                raise NoPathError(f"{label} {p} is inside a no-fly zone (including buffer)")
        if self.visible(start, goal):
            return [start, goal]

        n = len(self.nodes)
        s, g = n, n + 1
        pts = self.nodes + [start, goal]

        def neighbours(u: int):
            if u < n:
                yield from self.adj[u]
                if self.visible(pts[u], goal):
                    yield g, dist(pts[u], goal)
            elif u == s:
                for v in range(n):
                    if self.visible(start, pts[v]):
                        yield v, dist(start, pts[v])

        best = {s: 0.0}
        parent = {s: -1}
        frontier = [(dist(start, goal), s)]
        while frontier:
            _, u = heapq.heappop(frontier)
            if u == g:
                path = []
                while u != -1:
                    path.append(pts[u])
                    u = parent[u]
                return path[::-1]
            for v, w in neighbours(u):
                cand = best[u] + w
                if cand < best.get(v, float("inf")):
                    best[v], parent[v] = cand, u
                    # Straight-line distance never overestimates, so A* stays optimal.
                    heapq.heappush(frontier, (cand + dist(pts[v], goal), v))
        raise NoPathError(f"no obstacle-free path from {start} to {goal}")
