import itertools
import math
import random

import pytest

from uavplan.tour import nearest_neighbour, optimise, or_opt, tour_length, two_opt


def random_matrix(n, seed):
    rng = random.Random(seed)
    pts = [(rng.uniform(0, 1000), rng.uniform(0, 1000)) for _ in range(n)]
    return [[math.dist(a, b) for b in pts] for a in pts]


def brute_force(d):
    rest = range(1, len(d))
    return min(tour_length([0, *perm], d) for perm in itertools.permutations(rest))


def is_valid(tour, n):
    return tour[0] == 0 and sorted(tour) == list(range(n))


@pytest.mark.parametrize("seed", range(25))
def test_optimise_close_to_optimal(seed):
    d = random_matrix(8, seed)
    tour = optimise(d)
    assert is_valid(tour, 8)
    assert tour_length(tour, d) <= brute_force(d) * 1.02


@pytest.mark.parametrize("seed", range(10))
def test_each_stage_never_worse(seed):
    d = random_matrix(30, seed)
    nn = nearest_neighbour(d)
    two = two_opt(nn, d)
    orr = or_opt(two, d)
    assert is_valid(nn, 30) and is_valid(two, 30) and is_valid(orr, 30)
    assert tour_length(two, d) <= tour_length(nn, d) + 1e-9
    assert tour_length(orr, d) <= tour_length(two, d) + 1e-9


def test_points_on_a_circle_give_the_circle():
    n = 12
    pts = [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)) for k in range(n)]
    order = list(range(n))
    random.Random(1).shuffle(order)
    order.remove(0)
    pts = [pts[0]] + [pts[k] for k in order]
    d = [[math.dist(a, b) for b in pts] for a in pts]
    assert tour_length(optimise(d), d) == pytest.approx(2 * n * math.sin(math.pi / n))


def test_two_opt_removes_crossing():
    # Square visited in crossing order 0,2,1,3 -> 2-opt must uncross it.
    pts = [(0, 0), (1, 1), (1, 0), (0, 1)]
    d = [[math.dist(a, b) for b in pts] for a in pts]
    assert tour_length(two_opt([0, 1, 2, 3], d), d) == pytest.approx(4)
