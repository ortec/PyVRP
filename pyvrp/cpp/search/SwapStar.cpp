#include "SwapStar.h"

#include <cassert>

using pyvrp::Cost;
using pyvrp::search::Route;
using pyvrp::search::SwapStar;
using TWS = pyvrp::TimeWindowSegment;

void SwapStar::updateRemovalCosts(Route *R1, CostEvaluator const &costEvaluator)
{
    auto const currentCost = R1->penalisedCost(costEvaluator);
    auto const &vehicleType = data.vehicleType(R1->vehicleType());

    for (auto *U : *R1)
    {
        auto const twData = TWS::merge(data.durationMatrix(),
                                       R1->twsBefore(U->idx() - 1),
                                       R1->twsAfter(U->idx() + 1));

        Distance const dist = R1->distance()
                              + data.dist(p(U)->client(), n(U)->client())
                              - data.dist(p(U)->client(), U->client())
                              - data.dist(U->client(), n(U)->client());

        // Note: we don't account for size (fixed cost) and load (penalty) of
        // the route, since another node will be replaced
        auto const cost = costEvaluator.penalisedRouteCost(
            R1->size(), dist, R1->load(), twData.totalTimeWarp(), vehicleType);
        removalCosts(R1->idx(), U->client()) = cost - currentCost;
    }
}

void SwapStar::updateInsertionCost(Route *R,
                                   Route::Node *U,
                                   CostEvaluator const &costEvaluator)
{
    auto &insertPositions = cache(R->idx(), U->client());

    insertPositions = {};
    insertPositions.shouldUpdate = false;

    auto const currentCost = R->penalisedCost(costEvaluator);
    auto const &vehicleType = data.vehicleType(R->vehicleType());

    for (size_t idx = 0; idx != R->size() + 1; ++idx)
    {
        // Insert cost of U just after V (V -> U -> ...).
        auto const tws = TWS::merge(data.durationMatrix(),
                                    R->twsBefore(idx),
                                    U->route()->tws(U->idx()),
                                    R->twsAfter(idx + 1));

        auto *V = (*R)[idx];
        auto const dist = R->distance() + data.dist(V->client(), U->client())
                          + data.dist(U->client(), n(V)->client())
                          - data.dist(V->client(), n(V)->client());
        auto const cost = costEvaluator.penalisedRouteCost(
            R->size(), dist, R->load(), tws.totalTimeWarp(), vehicleType);

        insertPositions.maybeAdd(cost - currentCost, V);
    }
}

std::pair<Cost, Route::Node *> SwapStar::getBestInsertPoint(
    Route::Node *U, Route::Node *V, CostEvaluator const &costEvaluator)
{
    auto &best_ = cache(V->route()->idx(), U->client());

    if (best_.shouldUpdate)  // then we first update the insert positions
        updateInsertionCost(V->route(), U, costEvaluator);

    for (size_t idx = 0; idx != 3; ++idx)  // only OK if V is not adjacent
        if (best_.locs[idx] && best_.locs[idx] != V && n(best_.locs[idx]) != V)
            return std::make_pair(best_.costs[idx], best_.locs[idx]);

    // As a fallback option, we consider inserting in the place of V.
    auto const currentCost = V->route()->penalisedCost(costEvaluator);
    auto const &vehicleType = data.vehicleType(V->route()->vehicleType());
    auto const tws = TWS::merge(data.durationMatrix(),
                                V->route()->twsBefore(V->idx() - 1),
                                U->route()->tws(U->idx()),
                                V->route()->twsAfter(V->idx() + 1));

    Distance const dist = V->route()->distance()
                          + data.dist(p(V)->client(), U->client())
                          + data.dist(U->client(), n(V)->client())
                          - data.dist(p(V)->client(), n(V)->client());
    auto const cost = costEvaluator.penalisedRouteCost(V->route()->size(),
                                                       dist,
                                                       V->route()->load(),
                                                       tws.totalTimeWarp(),
                                                       vehicleType);

    return std::make_pair(cost - currentCost, p(V));
}

Cost SwapStar::evaluateMove(Route::Node *U,
                            Route::Node *V,
                            Route::Node *remove,
                            CostEvaluator const &costEvaluator)
{
    assert(V->route() == remove->route());
    assert(V != remove);

    auto const *route = V->route();
    auto const currentCost = route->penalisedCost(costEvaluator);
    auto const &vehicleType = data.vehicleType(route->vehicleType());

    auto const load = route->load() + data.location(U->client()).demand
                      - data.location(remove->client()).demand;

    if (V == p(remove))
    {
        // Special case: insert U in place of remove. Doing so removes edges
        // V -> remove -> n(remove), and adds V -> U -> n(remove).
        auto const dist = route->distance()
                          + data.dist(V->client(), U->client())
                          + data.dist(U->client(), n(remove)->client())
                          - data.dist(V->client(), remove->client())
                          - data.dist(remove->client(), n(remove)->client());

        auto const tws = TWS::merge(data.durationMatrix(),
                                    route->twsBefore(V->idx()),
                                    U->route()->tws(U->idx()),
                                    route->twsAfter(V->idx() + 2));

        auto const cost = costEvaluator.penalisedRouteCost(
            route->size(), dist, load, tws.totalTimeWarp(), vehicleType);
        return cost - currentCost;
    }
    else  // in non-adjacent parts of the route.
    {
        auto const current = data.dist(V->client(), n(V)->client())
                             + data.dist(p(remove)->client(), remove->client())
                             + data.dist(remove->client(), n(remove)->client());

        auto const proposed
            = data.dist(V->client(), U->client())
              + data.dist(U->client(), n(V)->client())
              + data.dist(p(remove)->client(), n(remove)->client());

        auto const dist = route->distance() + proposed - current;

        // For the time window segment, evaluation depends on whether insertion
        // of U comes before or after removal of V in the route
        if (V->idx() < remove->idx())
        {
            auto const tws
                = TWS::merge(data.durationMatrix(),
                             route->twsBefore(V->idx()),
                             U->route()->tws(U->idx()),
                             route->twsBetween(V->idx() + 1, remove->idx() - 1),
                             route->twsAfter(remove->idx() + 1));

            auto const cost = costEvaluator.penalisedRouteCost(
                route->size(), dist, load, tws.totalTimeWarp(), vehicleType);
            return cost - currentCost;
        }
        else
        {
            auto const tws
                = TWS::merge(data.durationMatrix(),
                             route->twsBefore(remove->idx() - 1),
                             route->twsBetween(remove->idx() + 1, V->idx()),
                             U->route()->tws(U->idx()),
                             route->twsAfter(V->idx() + 1));

            auto const cost = costEvaluator.penalisedRouteCost(
                route->size(), dist, load, tws.totalTimeWarp(), vehicleType);
            return cost - currentCost;
        }
    }
}

void SwapStar::init(Solution const &solution)
{
    LocalSearchOperator<Route>::init(solution);
    std::fill(updated.begin(), updated.end(), true);
}

Cost SwapStar::evaluate(Route *routeU,
                        Route *routeV,
                        CostEvaluator const &costEvaluator)
{
    best = {};

    if (updated[routeV->idx()])
    {
        updateRemovalCosts(routeV, costEvaluator);
        updated[routeV->idx()] = false;

        for (size_t idx = data.numDepots(); idx != data.numLocations(); ++idx)
            cache(routeV->idx(), idx).shouldUpdate = true;
    }

    if (updated[routeU->idx()])
    {
        updateRemovalCosts(routeU, costEvaluator);
        updated[routeV->idx()] = false;

        for (size_t idx = data.numDepots(); idx != data.numLocations(); ++idx)
            cache(routeU->idx(), idx).shouldUpdate = true;
    }

    for (auto *U : *routeU)
        for (auto *V : *routeV)
        {
            Cost deltaCost = 0;

            auto const uDemand = data.location(U->client()).demand;
            auto const vDemand = data.location(V->client()).demand;
            auto const loadDiff = uDemand - vDemand;

            deltaCost += costEvaluator.loadPenalty(routeU->load() - loadDiff,
                                                   routeU->capacity());
            deltaCost -= costEvaluator.loadPenalty(routeU->load(),
                                                   routeU->capacity());

            deltaCost += costEvaluator.loadPenalty(routeV->load() + loadDiff,
                                                   routeV->capacity());
            deltaCost -= costEvaluator.loadPenalty(routeV->load(),
                                                   routeV->capacity());

            deltaCost += removalCosts(routeU->idx(), U->client());
            deltaCost += removalCosts(routeV->idx(), V->client());

            if (deltaCost >= 0)  // an early filter on many moves, before doing
                continue;        // costly work determining insertion points

            auto [extraV, UAfter] = getBestInsertPoint(U, V, costEvaluator);
            deltaCost += extraV;

            if (deltaCost >= 0)  // continuing here avoids evaluating another
                continue;        // potentially costly insertion point below

            auto [extraU, VAfter] = getBestInsertPoint(V, U, costEvaluator);
            deltaCost += extraU;

            if (deltaCost < best.cost)
            {
                best.cost = deltaCost;

                best.U = U;
                best.UAfter = UAfter;

                best.V = V;
                best.VAfter = VAfter;
            }
        }

    // It is possible for positive delta costs to turn negative when we do a
    // complete evaluation. But in practice that almost never happens, and is
    // not worth spending time on.
    if (best.cost >= 0)
        return best.cost;

    // Now do a full evaluation of the proposed swap move. This includes
    // possible time warp penalties.
    return evaluateMove(best.V, best.VAfter, best.U, costEvaluator)
           + evaluateMove(best.U, best.UAfter, best.V, costEvaluator);
}

void SwapStar::apply(Route *U, Route *V) const
{
    assert(best.U);
    assert(best.UAfter);
    assert(best.V);
    assert(best.VAfter);

    U->remove(best.U->idx());
    V->remove(best.V->idx());

    V->insert(best.UAfter->idx() + 1, best.U);
    U->insert(best.VAfter->idx() + 1, best.V);
}

void SwapStar::update(Route *U) { updated[U->idx()] = true; }
