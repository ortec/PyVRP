#include "MoveTwoClientsReversed.h"
#include "Route.h"
#include "TimeWindowSegment.h"

#include <cassert>

using pyvrp::search::MoveTwoClientsReversed;
using TWS = pyvrp::TimeWindowSegment;

pyvrp::Cost MoveTwoClientsReversed::evaluate(
    Route::Node *U, Route::Node *V, pyvrp::CostEvaluator const &costEvaluator)
{
    if (U == n(V) || n(U) == V || n(U)->isDepot())
        return 0;

    assert(U->route() && V->route());
    auto *uRoute = U->route();
    auto *vRoute = V->route();

    auto const &vehTypeU = data.vehicleType(uRoute->vehicleType());
    auto const &vehTypeV = data.vehicleType(vRoute->vehicleType());

    auto const deltaDistU = data.dist(p(U)->client(), n(n(U))->client())
                            - uRoute->distBetween(U->idx() - 1, U->idx() + 2);
    auto const deltaDistV = data.dist(V->client(), n(U)->client())
                            + data.dist(n(U)->client(), U->client())
                            + data.dist(U->client(), n(V)->client())
                            - data.dist(V->client(), n(V)->client());

    if (uRoute != vRoute)
    {
        auto const currentCost = uRoute->penalisedCost(costEvaluator)
                                 + vRoute->penalisedCost(costEvaluator);

        auto const deltaLoad = uRoute->loadBetween(U->idx(), U->idx() + 1);

        // Compute lower bound for new cost based on size, distance and load
        RouteData uRouteData(uRoute->size() - 2,
                             uRoute->distance() + deltaDistU,
                             uRoute->load() - deltaLoad,
                             0,
                             0);

        RouteData vRouteData(vRoute->size() + 2,
                             vRoute->distance() + deltaDistV,
                             vRoute->load() + deltaLoad,
                             0,
                             0);

        auto const lbCostU = costEvaluator.penalisedCost(uRouteData, vehTypeU);
        auto const lbCostV = costEvaluator.penalisedCost(vRouteData, vehTypeV);

        if (lbCostU + lbCostV >= currentCost)
            return 0;

        // Add timing information for route to get actual cost
        auto uTWS = TWS::merge(data.durationMatrix(),
                               uRoute->twsBefore(U->idx() - 1),
                               uRoute->twsAfter(U->idx() + 2));
        uRouteData.timeWarp = uTWS.totalTimeWarp();
        uRouteData.duration = uTWS.duration();
        auto const costU = costEvaluator.penalisedCost(uRouteData, vehTypeU);

        // Small optimization, check intermediate bound
        if (costU + lbCostV >= currentCost)
            return 0;

        // Add time warp and actual duration for route V to get actual cost
        auto vTWS = TWS::merge(data.durationMatrix(),
                               vRoute->twsBefore(V->idx()),
                               uRoute->tws(U->idx() + 1),
                               uRoute->tws(U->idx()),
                               vRoute->twsAfter(V->idx() + 1));
        vRouteData.timeWarp = vTWS.totalTimeWarp();
        vRouteData.duration = vTWS.duration();
        auto const costV = costEvaluator.penalisedCost(vRouteData, vehTypeV);

        return costU + costV - currentCost;
    }
    else  // within same route
    {
        auto const currentCost = uRoute->penalisedCost(costEvaluator);

        // First compute bound based on size, dist and load
        RouteData routeData(uRoute->size(),
                            uRoute->distance() + deltaDistU + deltaDistV,
                            uRoute->load(),
                            0,
                            0);

        if (costEvaluator.penalisedCost(routeData, vehTypeU) >= currentCost)
            return 0;

        if (U->idx() < V->idx())
        {
            auto const uTWS
                = TWS::merge(data.durationMatrix(),
                             uRoute->twsBefore(U->idx() - 1),
                             uRoute->twsBetween(U->idx() + 2, V->idx()),
                             uRoute->tws(U->idx() + 1),
                             uRoute->tws(U->idx()),
                             uRoute->twsAfter(V->idx() + 1));
            routeData.timeWarp = uTWS.totalTimeWarp();
            routeData.duration = uTWS.duration();
        }
        else
        {
            auto const uTWS
                = TWS::merge(data.durationMatrix(),
                             uRoute->twsBefore(V->idx()),
                             uRoute->tws(U->idx() + 1),
                             uRoute->tws(U->idx()),
                             uRoute->twsBetween(V->idx() + 1, U->idx() - 1),
                             uRoute->twsAfter(U->idx() + 2));
            routeData.timeWarp = uTWS.totalTimeWarp();
            routeData.duration = uTWS.duration();
        }
        return costEvaluator.penalisedCost(routeData, vehTypeU) - currentCost;
    }
}

void MoveTwoClientsReversed::apply(Route::Node *U, Route::Node *V) const
{
    auto *X = n(U);  // copy since the insert below changes n(U)

    U->route()->remove(X->idx());
    U->route()->remove(U->idx());

    V->route()->insert(V->idx() + 1, U);
    V->route()->insert(V->idx() + 1, X);
}
