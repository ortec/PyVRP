#include "primitives.h"
#include "TimeWindowSegment.h"

#include <cassert>

using pyvrp::Cost;
using TWS = pyvrp::TimeWindowSegment;

Cost pyvrp::search::insertCost(Route::Node *U,
                               Route::Node *V,
                               ProblemData const &data,
                               CostEvaluator const &costEvaluator)
{
    if (!V->route() || U->isDepot())
        return 0;

    auto *route = V->route();
    auto const &client = data.location(U->client());

    Distance const deltaDist = data.dist(V->client(), U->client())
                               + data.dist(U->client(), n(V)->client())
                               - data.dist(V->client(), n(V)->client());

    auto const vTWS = TWS::merge(data.durationMatrix(),
                                 route->twsBefore(V->idx()),
                                 TWS(U->client(), client),
                                 route->twsAfter(V->idx() + 1));

    RouteData const routeData(route->size() + 1,
                              route->distance() + deltaDist,
                              route->load() + client.demand,
                              vTWS.totalTimeWarp());
    auto const cost = costEvaluator.penalisedCost(
        routeData, data.vehicleType(route->vehicleType()));
    return cost - client.prize - route->penalisedCost(costEvaluator);
}

Cost pyvrp::search::removeCost(Route::Node *U,
                               ProblemData const &data,
                               CostEvaluator const &costEvaluator)
{
    if (!U->route() || U->isDepot())
        return 0;

    auto *route = U->route();
    auto const &client = data.location(U->client());

    Distance const deltaDist = data.dist(p(U)->client(), n(U)->client())
                               - data.dist(p(U)->client(), U->client())
                               - data.dist(U->client(), n(U)->client());

    auto uTWS = TWS::merge(data.durationMatrix(),
                           route->twsBefore(U->idx() - 1),
                           route->twsAfter(U->idx() + 1));

    RouteData const routeData(route->size() - 1,
                              route->distance() + deltaDist,
                              route->load() - client.demand,
                              uTWS.totalTimeWarp());
    auto const cost = costEvaluator.penalisedCost(
        routeData, data.vehicleType(route->vehicleType()));
    return cost + client.prize - route->penalisedCost(costEvaluator);
}
