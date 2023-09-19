#include "TwoOpt.h"

#include "Route.h"
#include "TimeWindowSegment.h"

#include <cassert>

using pyvrp::Cost;
using pyvrp::search::TwoOpt;
using TWS = pyvrp::TimeWindowSegment;

Cost TwoOpt::evalWithinRoute(Route::Node *U,
                             Route::Node *V,
                             CostEvaluator const &costEvaluator) const
{
    assert(U->route() == V->route());
    auto *route = U->route();
    auto const &vehicleType = data.vehicleType(route->vehicleType());
    auto const currentCost = route->penalisedCost(costEvaluator);

    // Current situation is U -> n(U) -> ... -> V -> n(V). Proposed move is
    // U -> V -> p(V) -> ... -> n(U) -> n(V). This reverses the segment from
    // n(U) to V.
    Distance segmentReversalDistance = 0;  // reversal dist of n(U) -> ... -> V
    for (auto *node = V; node != n(U); node = p(node))
        segmentReversalDistance += data.dist(node->client(), p(node)->client());

    Distance const deltaDist = data.dist(U->client(), V->client())
                               + data.dist(n(U)->client(), n(V)->client())
                               + segmentReversalDistance
                               - data.dist(U->client(), n(U)->client())
                               - data.dist(V->client(), n(V)->client())
                               - route->distBetween(U->idx() + 1, V->idx());
    auto const dist = route->distance() + deltaDist;

    // First compute bound based on dist and load
    RouteData routeData(route->size(), dist, route->load(), 0);

    if (costEvaluator.penalisedCost(routeData, vehicleType) >= currentCost)
        return 0;

    // Compute time warp for route to get actual cost
    auto tws = route->twsBefore(U->idx());
    for (size_t idx = V->idx(); idx != U->idx(); --idx)
        tws = TWS::merge(data.durationMatrix(), tws, route->tws(idx));
    tws = TWS::merge(data.durationMatrix(), tws, route->twsAfter(V->idx() + 1));
    routeData.timeWarp = tws.totalTimeWarp();

    return costEvaluator.penalisedCost(routeData, vehicleType) - currentCost;
}

Cost TwoOpt::evalBetweenRoutes(Route::Node *U,
                               Route::Node *V,
                               CostEvaluator const &costEvaluator) const
{
    assert(U->route() && V->route());
    auto *uRoute = U->route();
    auto *vRoute = V->route();

    // Two routes. Current situation is U -> n(U), and V -> n(V). Proposed move
    // is U -> n(V) and V -> n(U).
    auto const currentCost = uRoute->penalisedCost(costEvaluator)
                             + vRoute->penalisedCost(costEvaluator);

    auto const &vehTypeU = data.vehicleType(uRoute->vehicleType());
    auto const &vehTypeV = data.vehicleType(vRoute->vehicleType());

    // Compute lower bound for new cost based on size, distance and load
    // ->idx() corresponds to size of route up to that node
    auto const sizeU = U->idx() + vRoute->size() - V->idx();
    auto const sizeV = V->idx() + uRoute->size() - U->idx();

    auto const distU = uRoute->distBetween(0, U->idx())
                       + data.dist(U->client(), n(V)->client())
                       + vRoute->distance()
                       - vRoute->distBetween(0, V->idx() + 1);
    auto const distV = vRoute->distBetween(0, V->idx())
                       + data.dist(V->client(), n(U)->client())
                       + uRoute->distance()
                       - uRoute->distBetween(0, U->idx() + 1);

    // Proposed move appends the segment after V to U, and the segment after U
    // to V. So we need to make a distinction between the loads at U and V, and
    // the loads from clients visited after these nodes.
    auto const curLoadUntilU = uRoute->loadBetween(0, U->idx());
    auto const curLoadAfterU = uRoute->load() - curLoadUntilU;
    auto const curLoadUntilV = vRoute->loadBetween(0, V->idx());
    auto const curLoadAfterV = vRoute->load() - curLoadUntilV;

    // Load for new routes
    auto const loadU = curLoadUntilU + curLoadAfterV;
    auto const loadV = curLoadUntilV + curLoadAfterU;

    RouteData uRouteData(sizeU, distU, loadU, 0);
    RouteData vRouteData(sizeV, distV, loadV, 0);

    auto const lbCostU = costEvaluator.penalisedCost(uRouteData, vehTypeU);
    auto const lbCostV = costEvaluator.penalisedCost(vRouteData, vehTypeV);

    if (lbCostU + lbCostV >= currentCost)
        return 0;

    // Add time warp for route U to get actual cost
    if (V->idx() < vRoute->size())
    {
        auto const uTWS
            = TWS::merge(data.durationMatrix(),
                         uRoute->twsBefore(U->idx()),
                         vRoute->twsBetween(V->idx() + 1, vRoute->size()),
                         uRoute->tws(uRoute->size() + 1));
        uRouteData.timeWarp = uTWS.totalTimeWarp();
    }
    else
    {
        auto const uTWS = TWS::merge(data.durationMatrix(),
                                     uRoute->twsBefore(U->idx()),
                                     uRoute->tws(uRoute->size() + 1));
        uRouteData.timeWarp = uTWS.totalTimeWarp();
    }
    auto const costU = costEvaluator.penalisedCost(uRouteData, vehTypeU);

    if (costU + lbCostV >= currentCost)
        return 0;

    if (U->idx() < uRoute->size())
    {
        auto const vTWS
            = TWS::merge(data.durationMatrix(),
                         vRoute->twsBefore(V->idx()),
                         uRoute->twsBetween(U->idx() + 1, uRoute->size()),
                         vRoute->tws(vRoute->size() + 1));
        vRouteData.timeWarp = vTWS.totalTimeWarp();
    }
    else
    {
        auto const vTWS = TWS::merge(data.durationMatrix(),
                                     vRoute->twsBefore(V->idx()),
                                     vRoute->tws(vRoute->size() + 1));
        vRouteData.timeWarp = vTWS.totalTimeWarp();
    }
    auto const costV = costEvaluator.penalisedCost(vRouteData, vehTypeV);

    return costU + costV - currentCost;
}

void TwoOpt::applyWithinRoute(Route::Node *U, Route::Node *V) const
{
    auto *nU = n(U);

    while (V->idx() > nU->idx())
    {
        auto *pV = p(V);
        Route::swap(nU, V);
        nU = n(V);  // after swap, V is now nU
        V = pV;
    }
}

void TwoOpt::applyBetweenRoutes(Route::Node *U, Route::Node *V) const
{
    auto *nU = n(U);
    auto *nV = n(V);

    auto insertIdx = U->idx() + 1;
    while (!nV->isDepot())
    {
        auto *node = nV;
        nV = n(nV);
        V->route()->remove(node->idx());
        U->route()->insert(insertIdx++, node);
    }

    insertIdx = V->idx() + 1;
    while (!nU->isDepot())
    {
        auto *node = nU;
        nU = n(nU);
        U->route()->remove(node->idx());
        V->route()->insert(insertIdx++, node);
    }
}

Cost TwoOpt::evaluate(Route::Node *U,
                      Route::Node *V,
                      CostEvaluator const &costEvaluator)
{
    if (U->route()->idx() > V->route()->idx())  // tackled in a later iteration
        return 0;

    if (U->route() != V->route())
        return evalBetweenRoutes(U, V, costEvaluator);

    if (U->idx() + 1 >= V->idx())  // tackled in a later iteration
        return 0;

    return evalWithinRoute(U, V, costEvaluator);
}

void TwoOpt::apply(Route::Node *U, Route::Node *V) const
{
    if (U->route() == V->route())
        applyWithinRoute(U, V);
    else
        applyBetweenRoutes(U, V);
}
