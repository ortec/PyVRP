#include "greedy_repair.h"
#include "repair.h"

#include "search/primitives.h"

#include <cassert>
#include <limits>

using pyvrp::Solution;
using pyvrp::search::insertCost;

using SearchRoute = pyvrp::search::Route;
using SolRoute = pyvrp::Route;

std::vector<SolRoute>
pyvrp::repair::greedyRepair(std::vector<SolRoute> const &solRoutes,
                            std::vector<size_t> const &unplanned,
                            ProblemData const &data,
                            CostEvaluator const &costEvaluator)
{
    if (solRoutes.empty() && !unplanned.empty())
        throw std::invalid_argument("Need routes to repair!");

    std::vector<SearchRoute::Node> locs;
    std::vector<SearchRoute> routes;
    setupRoutes(locs, routes, solRoutes, data);

    for (auto const client : unplanned)
    {
        SearchRoute::Node *U = &locs[client];
        assert(!U->route());

        SearchRoute::Node *UAfter = nullptr;
        pyvrp::Cost deltaCost = std::numeric_limits<pyvrp::Cost>::max();

        for (auto &route : routes)
        {
            for (auto *V : route)
            {
                if (V->isEndDepot() && route.numTrips() == route.maxTrips())
                    continue;

                auto const cost = insertCost(U, V, data, costEvaluator);
                if (cost < deltaCost)
                {
                    deltaCost = cost;
                    UAfter = V;
                }
            }
        }

        assert(UAfter && UAfter->route());
        UAfter->route()->insert(UAfter->idx() + 1, U);
        UAfter->route()->update();
    }

    return exportRoutes(data, routes);
}
