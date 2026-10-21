"""Waypoint ordering: a travelling-salesman problem on the obstacle-aware distance matrix.

Exact TSP is exponential, so we use the standard construct-then-improve recipe:

* nearest neighbour  -- greedy O(n^2) starting tour, typically ~25% above optimal
* 2-opt              -- reverse a section of the tour when that shortens it
                        (removes every self-crossing)
* Or-opt             -- move a chain of 1-3 consecutive stops to a better place
                        (catches what 2-opt can't, without reversing anything)

Tours are closed loops that start and end at index 0 (home). They are stored
without repeating the home index at the end.
"""
from __future__ import annotations

from typing import List, Sequence

Matrix = Sequence[Sequence[float]]


def tour_length(tour: Sequence[int], d: Matrix) -> float:
    return sum(d[tour[i]][tour[(i + 1) % len(tour)]] for i in range(len(tour)))


def nearest_neighbour(d: Matrix, start: int = 0) -> List[int]:
    unvisited = set(range(len(d))) - {start}
    tour = [start]
    while unvisited:
        here = tour[-1]
        nxt = min(unvisited, key=lambda j: (d[here][j], j))
        tour.append(nxt)
        unvisited.remove(nxt)
    return tour


def two_opt(tour: List[int], d: Matrix) -> List[int]:
    """Repeat until no improving move: replace edges (a,b),(c,e) with (a,c),(b,e).

    Equivalent to reversing tour[i..j]. Assumes a symmetric matrix. Position 0
    (home) never moves because i starts at 1.
    """
    tour = list(tour)
    n = len(tour)
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                a, b = tour[i - 1], tour[i]
                c, e = tour[j], tour[(j + 1) % n]
                delta = d[a][c] + d[b][e] - d[a][b] - d[c][e]
                if delta < -1e-9:
                    tour[i:j + 1] = reversed(tour[i:j + 1])
                    improved = True
    return tour


def or_opt(tour: List[int], d: Matrix, max_chain: int = 3) -> List[int]:
    """Move chains of 1..max_chain stops to the best other position, in either direction."""
    tour = list(tour)
    n = len(tour)
    improved = True
    while improved:
        improved = False
        for length in range(1, max_chain + 1):
            for i in range(1, n - length + 1):
                chain = tour[i:i + length]
                prev, nxt = tour[i - 1], tour[(i + length) % n]
                removed_gain = d[prev][chain[0]] + d[chain[-1]][nxt] - d[prev][nxt]
                rest = tour[:i] + tour[i + length:]
                best_delta, best = -1e-9, None
                for k in range(len(rest)):
                    u, v = rest[k], rest[(k + 1) % len(rest)]
                    for seg in (chain, chain[::-1]):
                        delta = d[u][seg[0]] + d[seg[-1]][v] - d[u][v] - removed_gain
                        if delta < best_delta:
                            best_delta, best = delta, rest[:k + 1] + list(seg) + rest[k + 1:]
                if best is not None:
                    tour = best
                    improved = True
                    break
            if improved:
                break
    return tour


def optimise(d: Matrix) -> List[int]:
    """NN construction, then alternate 2-opt and Or-opt until neither helps."""
    tour = nearest_neighbour(d)
    while True:
        before = tour_length(tour, d)
        tour = or_opt(two_opt(tour, d), d)
        if tour_length(tour, d) >= before - 1e-9:
            return tour
