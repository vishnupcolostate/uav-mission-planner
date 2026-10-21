"""Battery-limited sortie splitting.

A UAV can't always fly the whole tour on one battery. Walk the optimised
tour and, before each next stop, check that the aircraft could still fly
there *and back home* within its range. If not, return home, swap the
battery, and start a new sortie from the next stop.
"""
from __future__ import annotations

from typing import List, Optional, Sequence

from .tour import Matrix


class InfeasibleMissionError(RuntimeError):
    pass


def split_sorties(tour: Sequence[int], d: Matrix, max_range: float, home: int = 0,
                  labels: Optional[Sequence[str]] = None) -> List[List[int]]:
    """Return sorties as index lists, each starting and ending at home."""
    stops = [t for t in tour if t != home]
    for s in stops:
        if d[home][s] + d[s][home] > max_range:
            name = labels[s] if labels else f"stop {s}"
            raise InfeasibleMissionError(
                f"{name} needs {d[home][s] + d[s][home]:.0f} m round trip, range is {max_range:.0f} m")

    sorties: List[List[int]] = []
    current, flown = [home], 0.0
    for s in stops:
        here = current[-1]
        if flown + d[here][s] + d[s][home] > max_range:
            sorties.append(current + [home])
            current, flown = [home], 0.0
            here = home
        flown += d[here][s]
        current.append(s)
    sorties.append(current + [home])
    return sorties


def sortie_length(sortie: Sequence[int], d: Matrix) -> float:
    return sum(d[sortie[i]][sortie[i + 1]] for i in range(len(sortie) - 1))


def split_best_rotation(tour: Sequence[int], d: Matrix, max_range: float, home: int = 0,
                        labels: Optional[Sequence[str]] = None) -> List[List[int]]:
    """Route-first, cluster-second: try cutting the loop starting at every stop, both directions."""
    stops = [t for t in tour if t != home]
    best: Optional[List[List[int]]] = None
    for direction in (stops, stops[::-1]):
        for r in range(len(direction)):
            candidate = split_sorties([home] + direction[r:] + direction[:r], d, max_range, home, labels)
            if best is None or _total(candidate, d) < _total(best, d) - 1e-9:
                best = candidate
    return best or [[home, home]]


def clarke_wright(d: Matrix, max_range: float, home: int = 0) -> List[List[int]]:
    """Clarke-Wright savings heuristic for range-limited multi-trip routing.

    Start with one out-and-back sortie per stop. Joining the sortie that ends at
    i with the one that starts at j saves d[i][home] + d[home][j] - d[i][j].
    Apply joins in order of largest saving, as long as the result fits the range.
    """
    stops = [i for i in range(len(d)) if i != home]
    routes = {i: [i] for i in stops}          # route id -> stops (no home)
    route_of = {i: i for i in stops}
    savings = sorted(
        ((d[home][i] + d[home][j] - d[i][j], i, j) for i in stops for j in stops if i < j),
        reverse=True,
    )

    def length(r: List[int]) -> float:
        return d[home][r[0]] + sum(d[r[k]][r[k + 1]] for k in range(len(r) - 1)) + d[r[-1]][home]

    for saving, i, j in savings:
        if saving <= 0:
            break
        ri, rj = route_of[i], route_of[j]
        if ri == rj:
            continue
        a, b = routes[ri], routes[rj]
        # i and j must be route endpoints; orient so a ends with i and b starts with j.
        if a[0] == i:
            a = a[::-1]
        if b[-1] == j:
            b = b[::-1]
        if a[-1] != i or b[0] != j:
            continue
        merged = a + b
        if length(merged) > max_range:
            continue
        routes[ri] = merged
        del routes[rj]
        for s in b:
            route_of[s] = ri
    return [[home] + r + [home] for r in routes.values()]


def _total(sorties: Sequence[Sequence[int]], d: Matrix) -> float:
    return sum(sortie_length(s, d) for s in sorties)
