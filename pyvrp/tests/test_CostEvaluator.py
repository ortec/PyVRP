from typing import Tuple

import numpy as np
from numpy.testing import assert_, assert_allclose
from pytest import mark

from pyvrp import CostEvaluator, Route, Solution, VehicleType
from pyvrp._pyvrp import RouteData


def test_load_penalty():
    """
    This test asserts that load penalty computations are correct.
    """
    cost_evaluator = CostEvaluator(2, 1)

    assert_allclose(cost_evaluator.load_penalty(0, 1), 0)  # below capacity
    assert_allclose(cost_evaluator.load_penalty(1, 1), 0)  # at capacity

    # Penalty per unit excess capacity is 2
    # 1 unit above capacity
    assert_allclose(cost_evaluator.load_penalty(2, 1), 2)
    # 2 units above capacity
    assert_allclose(cost_evaluator.load_penalty(3, 1), 4)

    # Penalty per unit excess capacity is 4
    cost_evaluator = CostEvaluator(4, 1)

    # 1 unit above capacity
    assert_allclose(cost_evaluator.load_penalty(2, 1), 4)
    # 2 units above capacity
    assert_allclose(cost_evaluator.load_penalty(3, 1), 8)


@mark.parametrize("capacity", [5, 15, 29, 51, 103])
def test_load_penalty_always_zero_when_below_capacity(capacity: int):
    """
    This test asserts that load penalties are only applied to excess load, that
    is, load in excess of the vehicle's capacity.
    """
    load_penalty = 2
    cost_evaluator = CostEvaluator(load_penalty, 1)

    for load in range(capacity):  # all below capacity
        assert_allclose(cost_evaluator.load_penalty(load, capacity), 0)

    # at capacity
    assert_allclose(cost_evaluator.load_penalty(capacity, capacity), 0)
    # above capacity
    assert_allclose(
        cost_evaluator.load_penalty(capacity + 1, capacity), load_penalty
    )
    assert_allclose(
        cost_evaluator.load_penalty(capacity + 2, capacity), 2 * load_penalty
    )


def test_tw_penalty():
    """
    This test asserts that time window penalty computations are correct.
    """
    cost_evaluator = CostEvaluator(1, 2)

    # Penalty per unit time warp is 2
    assert_allclose(cost_evaluator.tw_penalty(0), 0)
    assert_allclose(cost_evaluator.tw_penalty(1), 2)
    assert_allclose(cost_evaluator.tw_penalty(2), 4)

    cost_evaluator = CostEvaluator(1, 4)

    # Penalty per unit excess capacity is now 4
    assert_allclose(cost_evaluator.tw_penalty(0), 0)
    assert_allclose(cost_evaluator.tw_penalty(1), 4)
    assert_allclose(cost_evaluator.tw_penalty(2), 8)


def test_cost(ok_small):
    """
    This test asserts that the cost is computed correctly for feasible
    solutions, and is a large value (representing infinity) for infeasible
    solutions.
    """
    default_cost_evaluator = CostEvaluator()
    cost_evaluator = CostEvaluator(20, 6)

    feas_sol = Solution(ok_small, [[1, 2], [3], [4]])  # feasible solution
    distance = feas_sol.distance()

    assert_allclose(cost_evaluator.cost(feas_sol), distance)
    assert_allclose(default_cost_evaluator.cost(feas_sol), distance)

    infeas_sol = Solution(ok_small, [[1, 2, 3, 4]])  # infeasible solution
    assert_(not infeas_sol.is_feasible())

    # The C++ code represents infinity using a relevant maximal value, which
    # depends on the precision type (double or integer).
    if isinstance(distance, int):
        INFEAS_COST = np.iinfo(np.int32).max
    else:
        INFEAS_COST = np.finfo(np.float64).max

    assert_allclose(cost_evaluator.cost(infeas_sol), INFEAS_COST)
    assert_allclose(default_cost_evaluator.cost(infeas_sol), INFEAS_COST)


def test_cost_with_prizes(prize_collecting):
    """
    When solving a prize-collecting instance, the cost is equal to the distance
    plus a prize term.
    """
    data = prize_collecting
    cost_evaluator = CostEvaluator(1, 1)

    sol = Solution(data, [[1, 2], [3, 4, 5]])
    cost = cost_evaluator.cost(sol)
    assert_(sol.is_feasible())

    prizes = [client.prize for client in data.clients()]
    collected = sum(prizes[:5])
    uncollected = sum(prizes) - collected

    assert_allclose(sol.prizes(), collected)
    assert_allclose(sol.uncollected_prizes(), uncollected)
    assert_allclose(sol.distance() + sol.uncollected_prizes(), cost)


def test_penalised_cost(ok_small):
    """
    The penalised cost represents the smoothed objective, where constraint
    violations are priced in using penalty terms. It can be computed for both
    feasible and infeasible solutions. In case of the former, it is equal
    to the actual cost: the penalty terms are all zero.
    """
    penalty_capacity = 20
    penalty_tw = 6
    default_evaluator = CostEvaluator()
    cost_evaluator = CostEvaluator(penalty_capacity, penalty_tw)

    feas = Solution(ok_small, [[1, 2], [3], [4]])
    assert_(feas.is_feasible())

    # For a feasible solution, cost and penalised_cost equal distance.
    assert_allclose(cost_evaluator.penalised_cost(feas), feas.distance())
    assert_allclose(default_evaluator.penalised_cost(feas), feas.distance())

    infeas = Solution(ok_small, [[1, 2, 3, 4]])
    assert_(not infeas.is_feasible())

    # Compute cost associated with violated constraints.
    load_penalty_cost = penalty_capacity * infeas.excess_load()
    tw_penalty_cost = penalty_tw * infeas.time_warp()
    infeas_dist = infeas.distance()

    # Test penalised cost
    expected_cost = infeas_dist + load_penalty_cost + tw_penalty_cost
    assert_allclose(cost_evaluator.penalised_cost(infeas), expected_cost)

    # Default cost evaluator has 0 weights and only computes distance as cost
    assert_allclose(default_evaluator.penalised_cost(infeas), infeas_dist)


def test_route_penalised_cost():
    """
    The penalised cost represents the smoothed objective, where constraint
    violations are priced in using penalty terms. This test tests the overload
    that computes penalised cost for a specific route based on a
    pyvrp::RouteData object.
    """
    penalty_capacity = 20
    penalty_tw = 6
    cost_evaluator = CostEvaluator(penalty_capacity, penalty_tw)

    route_data = RouteData(
        size=1, distance=10, load=5, duration=0, time_warp=0
    )
    vehicle_type = VehicleType(10, 1, 100)

    # No load violations and time warp, so we should have just fixed cost
    # and distance = 100 + 10 = 110
    cost = cost_evaluator.penalised_cost(route_data, vehicle_type)
    assert_allclose(cost, 110)

    route_data = RouteData(
        size=1, distance=10, load=5, duration=0, time_warp=20
    )
    vehicle_type = VehicleType(3, 1, 100)
    # Now we should incur load penalty 20 * (5-3) = 40 and timewarp penalty
    # 6 * 20 = 120, beyond the fixed and distance cost so 110 + 40 + 120 = 270
    cost = cost_evaluator.penalised_cost(route_data, vehicle_type)
    assert_allclose(cost, 270)

    # Test empty route
    route_data = RouteData(size=0, distance=0, load=0, duration=0, time_warp=0)
    cost = cost_evaluator.penalised_cost(route_data, vehicle_type)
    assert_allclose(cost, 0)


@mark.parametrize(
    ("assignment", "expected"), [((0, 0), 0), ((0, 1), 10), ((1, 1), 20)]
)
def test_cost_with_fixed_vehicle_cost(
    ok_small, assignment: Tuple[int, int], expected: int
):
    """
    Tests that the cost evaluator counts the fixed cost when determining the
    objective value of a solution.
    """
    # First vehicle type is free, second costs 10 per vehicle. The solution
    # should be able to track this.
    data = ok_small.replace(
        vehicle_types=[
            VehicleType(10, 2, fixed_cost=0),
            VehicleType(10, 2, fixed_cost=10),
        ]
    )

    routes = [
        Route(data, [1, 2], assignment[0]),
        Route(data, [3, 4], assignment[1]),
    ]

    sol = Solution(data, routes)
    cost_eval = CostEvaluator(1, 1)

    # Solution is feasible, so penalised cost and regular cost are equal. Both
    # should contain the fixed vehicle cost.
    assert_(sol.is_feasible())
    assert_allclose(cost_eval.cost(sol), sol.distance() + expected)
    assert_allclose(cost_eval.penalised_cost(sol), sol.distance() + expected)


@mark.parametrize(
    ("assignment", "expected"),
    [((0, 0), 32175), ((0, 1), 50641), ((1, 1), 74095)],
)
def test_cost_with_heterogeneous_vehicle_cost(
    ok_small, assignment: Tuple[int, int], expected: int
):
    """
    Tests that the cost evaluator counts the fixed cost when determining the
    objective value of a solution.
    """
    # First vehicle type is free, second costs 10 per vehicle. The solution
    # should be able to track this.
    data = ok_small.replace(
        vehicle_types=[
            VehicleType(
                10, 2, fixed_cost=0, cost_per_distance=1, cost_per_duration=2
            ),
            VehicleType(
                10, 2, fixed_cost=10, cost_per_distance=3, cost_per_duration=4
            ),
        ]
    )

    routes = [
        Route(data, [1, 2], assignment[0]),  # distance 5501, duration 6221
        Route(data, [3, 4], assignment[1]),  # distance 4224, duration 5004
    ]

    sol = Solution(data, routes)
    cost_eval = CostEvaluator(1, 1)

    # Solution is feasible, so penalised cost and regular cost are equal. Both
    # should contain the fixed vehicle cost, distance cost and duration cost.
    assert_(sol.is_feasible())
    assert_allclose(cost_eval.cost(sol), expected)
    assert_allclose(cost_eval.penalised_cost(sol), expected)
