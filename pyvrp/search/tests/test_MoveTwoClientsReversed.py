import numpy as np
from numpy.testing import assert_, assert_allclose, assert_equal
from pytest import mark

from pyvrp import (
    Client,
    CostEvaluator,
    ProblemData,
    RandomNumberGenerator,
    Solution,
    VehicleType,
)
from pyvrp import Route as SolRoute
from pyvrp.search import (
    LocalSearch,
    MoveTwoClientsReversed,
    NeighbourhoodParams,
    compute_neighbours,
)
from pyvrp.search._search import LocalSearch as cpp_LocalSearch
from pyvrp.search._search import Node, Route


def test_single_route_OkSmall(ok_small):
    """
    This test checks that MoveTwoClientsReversed properly solves the small
    instance where we know what is going on.
    """
    cost_evaluator = CostEvaluator(20, 6)

    # Only neighbours are 1 -> 4 and 2 -> 1.
    ls = cpp_LocalSearch(ok_small, [[], [4], [1], [], []])
    ls.add_node_operator(MoveTwoClientsReversed(ok_small))

    sol = Solution(ok_small, [[1, 4, 2, 3]])
    improved_sol = ls.search(sol, cost_evaluator)

    # The new solution should strictly improve on our original solution.
    assert_equal(improved_sol.num_routes(), 1)
    current_cost = cost_evaluator.penalised_cost(sol)
    improved_cost = cost_evaluator.penalised_cost(improved_sol)
    assert_(improved_cost < current_cost)

    # (2, 3) was inserted after 1 as 1 -> 3 -> 2 -> 4. Then (1, 3) got inserted
    # after 4 as 2 -> 4 -> 3 -> 1.
    expected = Solution(ok_small, [[2, 4, 3, 1]])
    assert_equal(improved_sol, expected)

    # These two-route solutions can all be created by MoveTwoClientsReversed
    # from the returned solution. So they must have a cost that's at best equal
    # to the returned solution's cost.
    for routes in ([[3, 1], [4, 2]], [[2, 1], [3, 4]], [[2, 4], [1, 3]]):
        other = Solution(ok_small, routes)
        improved_cost = cost_evaluator.penalised_cost(improved_sol)
        other_cost = cost_evaluator.penalised_cost(other)
        assert_(improved_cost <= other_cost)


def test_OkSmall_with_fixed_clients(ok_small):
    """
    This test checks that MoveTwoClientsReversed cannot find improving moves
    when there is no segment of 2 clients without fixed clients.
    """
    data = ok_small.replace(
        clients=[
            Client(
                x=client.x,
                y=client.y,
                demand=client.demand,
                fixed_vehicle_type=veh_type,
            )
            for client, veh_type in zip(ok_small.clients(), (0, 1, None, None))
        ],
        vehicle_types=[
            VehicleType(10, 1),
            VehicleType(10, 1),
            VehicleType(10, 1),
        ],
    )
    cost_evaluator = CostEvaluator(20, 6)
    rng = RandomNumberGenerator(seed=42)

    nb_params = NeighbourhoodParams(nb_granular=data.num_clients)
    ls = LocalSearch(data, rng, compute_neighbours(data, nb_params))
    ls.add_node_operator(MoveTwoClientsReversed(data))

    solution = Solution(
        data, [SolRoute(data, [1, 4], 0), SolRoute(data, [3, 2], 1)]
    )
    assert_equal(solution, ls.search(solution, cost_evaluator))


@mark.parametrize("seed", [2643, 2742, 2941, 3457, 4299, 4497, 6178, 6434])
def test_RC208_instance(rc208, seed: int):
    """
    Test a larger instance over several seeds.
    """
    cost_evaluator = CostEvaluator(20, 6)
    rng = RandomNumberGenerator(seed=seed)

    nb_params = NeighbourhoodParams(nb_granular=rc208.num_clients)
    ls = LocalSearch(rc208, rng, compute_neighbours(rc208, nb_params))
    ls.add_node_operator(MoveTwoClientsReversed(rc208))

    single_route = list(range(rc208.num_depots, rc208.num_locations))
    sol = Solution(rc208, [single_route])
    improved_sol = ls.search(sol, cost_evaluator)

    # The new solution should strictly improve on our original solution.
    current_cost = cost_evaluator.penalised_cost(sol)
    improved_cost = cost_evaluator.penalised_cost(improved_sol)
    assert_(improved_cost < current_cost)


def test_relocate_fixed_vehicle_cost():
    """
    Tests that when relocating two clients leaves a route empty, into a new
    empty route correctly sums up the fixed vehicle cost if that's the only
    cost change.
    """
    data = ProblemData(
        clients=[Client(x=1, y=1), Client(x=1, y=0)],
        depots=[Client(x=0, y=0)],
        vehicle_types=[VehicleType(0, 1, 7), VehicleType(0, 1, 13)],
        distance_matrix=np.zeros((3, 3), dtype=int),
        duration_matrix=np.zeros((3, 3), dtype=int),
    )

    route1 = Route(data, idx=0, vehicle_type=0)
    route2 = Route(data, idx=1, vehicle_type=1)

    for loc in [1, 2]:
        route1.append(Node(loc=loc))

    route1.update()
    route2.update()

    # All distances, durations, and loads are equal. So the only cost change
    # can happen due to vehicle changes. In particular, we evaluate inserting
    # the two clients on route1 into the empty route2. That leaves route1 empty
    # and has a fixed vehicle cost change of 13 - 7 = 6.
    op = MoveTwoClientsReversed(data)
    cost_eval = CostEvaluator(0, 0)
    assert_allclose(op.evaluate(route1[1], route2[0], cost_eval), 6)
