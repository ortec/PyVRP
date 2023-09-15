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
        // Compute lower bound for new cost based on clients, distance and load
        auto const sizeU = uRoute->size() - 2;
        auto const sizeV = vRoute->size() + 2;

        auto const distU = uRoute->distance() + deltaDistU;
        auto const distV = vRoute->distance() + deltaDistV;

        auto const deltaLoad = uRoute->loadBetween(U->idx(), U->idx() + 1);
        auto const loadU = uRoute->load() - deltaLoad;
        auto const loadV = vRoute->load() + deltaLoad;

        auto const lbCostU = costEvaluator.penalisedRouteCost(
            sizeU, distU, loadU, 0, vehTypeU);
        auto const lbCostV = costEvaluator.penalisedRouteCost(
            sizeV, distV, loadV, 0, vehTypeV);

        if (lbCostU + lbCostV >= currentCost)
            return 0;

        // Add timing information for route to get actual cost
        auto uTWS = TWS::merge(data.durationMatrix(),
                               uRoute->twsBefore(U->idx() - 1),
                               uRoute->twsAfter(U->idx() + 2));
        auto const costU = costEvaluator.penalisedRouteCost(
            sizeU, distU, loadU, uTWS.totalTimeWarp(), vehTypeU);

        // Small optimization, check intermediate bound
        if (costU + lbCostV >= currentCost)
            return 0;

        // Add time warp and actual duration for route V to get actual cost
        auto vTWS = TWS::merge(data.durationMatrix(),
                               vRoute->twsBefore(V->idx()),
                               uRoute->tws(U->idx() + 1),
                               uRoute->tws(U->idx()),
                               vRoute->twsAfter(V->idx() + 1));
        auto const costV = costEvaluator.penalisedRouteCost(
            sizeV, distV, loadV, vTWS.totalTimeWarp(), vehTypeV);

        return costU + costV - currentCost;
    }
    else  // within same route
    {
        auto const currentCost = uRoute->penalisedCost(costEvaluator);
        auto const dist = uRoute->distance() + deltaDistU + deltaDistV;

        // First compute bound based on dist and load
        auto const lbCost = costEvaluator.penalisedRouteCost(
            uRoute->size(), dist, uRoute->load(), 0, vehTypeU);
        if (lbCost >= currentCost)
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

            auto const cost
                = costEvaluator.penalisedRouteCost(uRoute->size(),
                                                   dist,
                                                   uRoute->load(),
                                                   uTWS.totalTimeWarp(),
                                                   vehTypeU);
            return cost - currentCost;
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

            auto const cost
                = costEvaluator.penalisedRouteCost(uRoute->size(),
                                                   dist,
                                                   uRoute->load(),
                                                   uTWS.totalTimeWarp(),
                                                   vehTypeU);
            return cost - currentCost;
        }
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
