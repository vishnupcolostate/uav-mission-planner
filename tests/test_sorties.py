import math
import random

import pytest

from uavplan.sorties import (InfeasibleMissionError, clarke_wright, sortie_length,
                             split_best_rotation, split_sorties)
from uavplan.tour import optimise


def instance(n, seed):
    rng = random.Random(seed)
    pts = [(0.0, 0.0)] + [(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)) for _ in range(n)]
    return [[math.dist(a, b) for b in pts] for a in pts]


def check(sorties, d, max_range):
    visited = [s for sortie in sorties for s in sortie[1:-1]]
    assert sorted(visited) == list(range(1, len(d)))      # every stop exactly once
    for sortie in sorties:
        assert sortie[0] == sortie[-1] == 0               # starts and ends at home
        assert sortie_length(sortie, d) <= max_range + 1e-6


@pytest.mark.parametrize("seed", range(15))
@pytest.mark.parametrize("max_range", [9000, 15000, 30000])
def test_all_strategies_respect_range(seed, max_range):
    d = instance(15, seed)
    tour = optimise(d)
    check(split_sorties(tour, d, max_range), d, max_range)
    check(split_best_rotation(tour, d, max_range), d, max_range)
    check(clarke_wright(d, max_range), d, max_range)


def test_unlimited_range_is_one_sortie():
    d = instance(10, 0)
    assert len(split_sorties(optimise(d), d, 1e9)) == 1
    assert len(clarke_wright(d, 1e9)) == 1


def test_best_rotation_never_worse_than_plain_split():
    for seed in range(15):
        d = instance(15, seed)
        tour = optimise(d)
        plain = sum(sortie_length(s, d) for s in split_sorties(tour, d, 12000))
        best = sum(sortie_length(s, d) for s in split_best_rotation(tour, d, 12000))
        assert best <= plain + 1e-9


def test_out_of_range_stop_is_reported_by_name():
    d = [[0, 10, 100], [10, 0, 95], [100, 95, 0]]
    with pytest.raises(InfeasibleMissionError, match="FarAway"):
        split_sorties([0, 1, 2], d, 150, labels=["HOME", "Near", "FarAway"])
