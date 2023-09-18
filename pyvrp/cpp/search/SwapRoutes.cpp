#include "SwapRoutes.h"

#include <vector>

using pyvrp::Cost;
using pyvrp::search::Route;
using pyvrp::search::SwapRoutes;
using TWS = pyvrp::TimeWindowSegment;

Cost SwapRoutes::evaluate(Route *U,
                          Route *V,
                          CostEvaluator const &costEvaluator)
{
    if (U->vehicleType() == V->vehicleType() || U->empty() || V->empty())
        return 0;

    auto const currentCost
        = U->penalisedCost(costEvaluator) + V->penalisedCost(costEvaluator);
    auto const &vehTypeU = data.vehicleType(U->vehicleType());
    auto const &vehTypeV = data.vehicleType(V->vehicleType());

    auto const lbCostU = costEvaluator.penalisedRouteCost(
        U->size(), U->distance(), U->load(), 0, vehTypeV);
    auto const lbCostV = costEvaluator.penalisedRouteCost(
        V->size(), V->distance(), V->load(), 0, vehTypeU);

    if (lbCostU + lbCostV >= currentCost)
        return 0;

    // Changes in time warp.
    auto const uTWS = TWS::merge(data.durationMatrix(),
                                 V->tws(0),
                                 U->twsBetween(1, U->size()),
                                 V->tws(V->size() + 1));

    auto const costU = costEvaluator.penalisedRouteCost(
        U->size(), U->distance(), U->load(), uTWS.totalTimeWarp(), vehTypeV);

    if (costU + lbCostV >= currentCost)
        return 0;

    auto const vTWS = TWS::merge(data.durationMatrix(),
                                 U->tws(0),
                                 V->twsBetween(1, V->size()),
                                 U->tws(U->size() + 1));

    auto const costV = costEvaluator.penalisedRouteCost(
        V->size(), V->distance(), V->load(), vTWS.totalTimeWarp(), vehTypeU);

    // TODO handle the case of depot differences (multiple depots). There is
    // some evaluation code for this in issue #188.
    return costU + costV - currentCost;
}

void SwapRoutes::apply(Route *U, Route *V) const
{
    std::vector<Route::Node *> uNodes = {U->begin(), U->end()};
    std::vector<Route::Node *> vNodes = {V->begin(), V->end()};

    // We are swapping the routes completely, so we can just clear both.
    U->clear();
    V->clear();

    // Reinsert the nodes from route U in route V.
    for (auto *node : uNodes)
        V->push_back(node);

    // Reinsert the nodes from route V in route U.
    for (auto *node : vNodes)
        U->push_back(node);
}

SwapRoutes::SwapRoutes(ProblemData const &data)
    : LocalSearchOperator<Route>(data)
{
}
