#ifndef PYVRP_EXCHANGE_H
#define PYVRP_EXCHANGE_H

#include "LocalSearchOperator.h"
#include "TimeWindowSegment.h"

#include <cassert>

namespace pyvrp::search
{
/**
 * Exchange(data: ProblemData)
 *
 * The :math:`(N, M)`-exchange operators exchange :math:`N` consecutive clients
 * from :math:`U`'s route (starting at :math:`U`) with :math:`M` consecutive
 * clients from :math:`V`'s route (starting at :math:`V`). This includes
 * the RELOCATE and SWAP operators as special cases.
 *
 * The :math:`(N, M)`-exchange class uses C++ templates for different :math:`N`
 * and :math:`M` to efficiently evaluate these moves.
 */
template <size_t N, size_t M>
class Exchange : public LocalSearchOperator<Route::Node>
{
    using LocalSearchOperator::LocalSearchOperator;

    static_assert(N >= M && N > 0, "N < M or N == 0 does not make sense");

    // Tests if the segment starting at node of given length contains the depot
    inline bool containsDepot(Route::Node *node, size_t segLength) const;

    // Tests if the segments of U and V overlap in the same route
    inline bool overlap(Route::Node *U, Route::Node *V) const;

    // Tests if the segments of U and V are adjacent in the same route
    inline bool adjacent(Route::Node *U, Route::Node *V) const;

    // Special case that's applied when M == 0
    Cost evalRelocateMove(Route::Node *U,
                          Route::Node *V,
                          CostEvaluator const &costEvaluator) const;

    // Applied when M != 0
    Cost evalSwapMove(Route::Node *U,
                      Route::Node *V,
                      CostEvaluator const &costEvaluator) const;

public:
    Cost evaluate(Route::Node *U,
                  Route::Node *V,
                  CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U, Route::Node *V) const override;
};

template <size_t N, size_t M>
bool Exchange<N, M>::containsDepot(Route::Node *node, size_t segLength) const
{
    // size() is the position of the last client in the route. So the segment
    // must include the depot if idx + move length - 1 (-1 since we're also
    // moving the node *at* idx) is larger than size().
    return node->isDepot()
           || (node->idx() + segLength - 1 > node->route()->size());
}

template <size_t N, size_t M>
bool Exchange<N, M>::overlap(Route::Node *U, Route::Node *V) const
{
    return U->route() == V->route()
           // We need max(M, 1) here because when V is the depot and M == 0,
           // this would turn negative and wrap around to a large number.
           && U->idx() <= V->idx() + std::max<size_t>(M, 1) - 1
           && V->idx() <= U->idx() + N - 1;
}

template <size_t N, size_t M>
bool Exchange<N, M>::adjacent(Route::Node *U, Route::Node *V) const
{
    return U->route() == V->route()
           && (U->idx() + N == V->idx() || V->idx() + M == U->idx());
}

template <size_t N, size_t M>
Cost Exchange<N, M>::evalRelocateMove(Route::Node *U,
                                      Route::Node *V,
                                      CostEvaluator const &costEvaluator) const
{
    assert(U->idx() > 0);

    auto *uRoute = U->route();
    auto *vRoute = V->route();

    auto const &vehTypeU = data.vehicleType(uRoute->vehicleType());
    auto const &vehTypeV = data.vehicleType(vRoute->vehicleType());

    auto *endU = N == 1 ? U : (*uRoute)[U->idx() + N - 1];

    auto const deltaDistU = data.dist(p(U)->client(), n(endU)->client())
                            - uRoute->distBetween(U->idx() - 1, U->idx() + N);
    auto const deltaDistV = data.dist(V->client(), U->client())
                            + uRoute->distBetween(U->idx(), U->idx() + N - 1)
                            + data.dist(endU->client(), n(V)->client())
                            - data.dist(V->client(), n(V)->client());

    if (uRoute != vRoute)
    {
        auto const currentCost = uRoute->penalisedCost(costEvaluator)
                                 + vRoute->penalisedCost(costEvaluator);

        auto const deltaLoad = uRoute->loadBetween(U->idx(), U->idx() + N - 1);

        // Compute lower bound for new cost based on size, distance and load
        RouteData uRouteData(uRoute->size() - N,
                             uRoute->distance() + deltaDistU,
                             uRoute->load() - deltaLoad,
                             0);

        RouteData vRouteData(vRoute->size() + N,
                             vRoute->distance() + deltaDistV,
                             vRoute->load() + deltaLoad,
                             0);

        auto const lbCostU = costEvaluator.penalisedCost(uRouteData, vehTypeU);
        auto const lbCostV = costEvaluator.penalisedCost(vRouteData, vehTypeV);

        if (lbCostU + lbCostV >= currentCost)
            return 0;

        // Add time warp for route U to get actual cost
        auto uTWS = TimeWindowSegment::merge(data.durationMatrix(),
                                             uRoute->twsBefore(U->idx() - 1),
                                             uRoute->twsAfter(U->idx() + N));
        uRouteData.timeWarp = uTWS.totalTimeWarp();
        auto const costU = costEvaluator.penalisedCost(uRouteData, vehTypeU);

        // Small optimization, check intermediate bound
        if (costU + lbCostV >= currentCost)
            return 0;

        auto vTWS = TimeWindowSegment::merge(
            data.durationMatrix(),
            vRoute->twsBefore(V->idx()),
            uRoute->twsBetween(U->idx(), U->idx() + N - 1),
            vRoute->twsAfter(V->idx() + 1));
        vRouteData.timeWarp = vTWS.totalTimeWarp();
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
                            0);

        if (costEvaluator.penalisedCost(routeData, vehTypeU) >= currentCost)
            return 0;

        // Add timing information for route to get actual cost
        if (U->idx() < V->idx())
        {
            auto const tws = TimeWindowSegment::merge(
                data.durationMatrix(),
                uRoute->twsBefore(U->idx() - 1),
                uRoute->twsBetween(U->idx() + N, V->idx()),
                uRoute->twsBetween(U->idx(), U->idx() + N - 1),
                uRoute->twsAfter(V->idx() + 1));
            routeData.timeWarp = tws.totalTimeWarp();
        }
        else
        {
            auto const tws = TimeWindowSegment::merge(
                data.durationMatrix(),
                uRoute->twsBefore(V->idx()),
                uRoute->twsBetween(U->idx(), U->idx() + N - 1),
                uRoute->twsBetween(V->idx() + 1, U->idx() - 1),
                uRoute->twsAfter(U->idx() + N));
            routeData.timeWarp = tws.totalTimeWarp();
        }
        return costEvaluator.penalisedCost(routeData, vehTypeU) - currentCost;
    }
}

template <size_t N, size_t M>
Cost Exchange<N, M>::evalSwapMove(Route::Node *U,
                                  Route::Node *V,
                                  CostEvaluator const &costEvaluator) const
{
    assert(U->idx() > 0 && V->idx() > 0);
    assert(U->route() && V->route());

    auto *uRoute = U->route();
    auto *vRoute = V->route();

    auto const &vehTypeU = data.vehicleType(uRoute->vehicleType());
    auto const &vehTypeV = data.vehicleType(vRoute->vehicleType());

    auto *endU = N == 1 ? U : (*uRoute)[U->idx() + N - 1];
    auto *endV = M == 1 ? V : (*vRoute)[V->idx() + M - 1];

    //   p(U) -> V -> ... -> endV -> n(endU)
    // - p(U) -> ... -> n(endU)
    auto const deltaDistU = data.dist(p(U)->client(), V->client())
                            + vRoute->distBetween(V->idx(), V->idx() + M - 1)
                            + data.dist(endV->client(), n(endU)->client())
                            - uRoute->distBetween(U->idx() - 1, U->idx() + N);

    // + p(V) -> U -> ... -> endU -> n(endV)
    // - p(V) -> ... -> n(endV)
    auto const deltaDistV = data.dist(p(V)->client(), U->client())
                            + uRoute->distBetween(U->idx(), U->idx() + N - 1)
                            + data.dist(endU->client(), n(endV)->client())
                            - vRoute->distBetween(V->idx() - 1, V->idx() + M);

    if (uRoute != vRoute)
    {
        auto const currentCost = uRoute->penalisedCost(costEvaluator)
                                 + vRoute->penalisedCost(costEvaluator);

        auto const deltaLoad
            = uRoute->loadBetween(U->idx(), U->idx() + N - 1)
              - vRoute->loadBetween(V->idx(), V->idx() + M - 1);

        // Compute lower bound for new cost based on size, distance and load
        RouteData uRouteData(uRoute->size() - N + M,
                             uRoute->distance() + deltaDistU,
                             uRoute->load() - deltaLoad,
                             0);

        RouteData vRouteData(vRoute->size() + N - M,
                             vRoute->distance() + deltaDistV,
                             vRoute->load() + deltaLoad,
                             0);

        auto const lbCostU = costEvaluator.penalisedCost(uRouteData, vehTypeU);
        auto const lbCostV = costEvaluator.penalisedCost(vRouteData, vehTypeV);

        if (lbCostU + lbCostV >= currentCost)
            return 0;

        // Add timing information for route U to get actual cost
        auto uTWS = TimeWindowSegment::merge(
            data.durationMatrix(),
            uRoute->twsBefore(U->idx() - 1),
            vRoute->twsBetween(V->idx(), V->idx() + M - 1),
            uRoute->twsAfter(U->idx() + N));
        uRouteData.timeWarp = uTWS.totalTimeWarp();
        auto const costU = costEvaluator.penalisedCost(uRouteData, vehTypeU);

        // Small optimization, check intermediate bound
        if (costU + lbCostV >= currentCost)
            return 0;

        // Add timing information for route V to get actual cost
        auto vTWS = TimeWindowSegment::merge(
            data.durationMatrix(),
            vRoute->twsBefore(V->idx() - 1),
            uRoute->twsBetween(U->idx(), U->idx() + N - 1),
            vRoute->twsAfter(V->idx() + M));
        vRouteData.timeWarp = vTWS.totalTimeWarp();
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
                            0);

        if (costEvaluator.penalisedCost(routeData, vehTypeU) >= currentCost)
            return 0;

        if (U->idx() < V->idx())
        {
            auto const tws = TimeWindowSegment::merge(
                data.durationMatrix(),
                uRoute->twsBefore(U->idx() - 1),
                uRoute->twsBetween(V->idx(), V->idx() + M - 1),
                uRoute->twsBetween(U->idx() + N, V->idx() - 1),
                uRoute->twsBetween(U->idx(), U->idx() + N - 1),
                uRoute->twsAfter(V->idx() + M));
            routeData.timeWarp = tws.totalTimeWarp();
        }
        else
        {
            auto const tws = TimeWindowSegment::merge(
                data.durationMatrix(),
                uRoute->twsBefore(V->idx() - 1),
                uRoute->twsBetween(U->idx(), U->idx() + N - 1),
                uRoute->twsBetween(V->idx() + M, U->idx() - 1),
                uRoute->twsBetween(V->idx(), V->idx() + M - 1),
                uRoute->twsAfter(U->idx() + N));
            routeData.timeWarp = tws.totalTimeWarp();
        }
        return costEvaluator.penalisedCost(routeData, vehTypeU) - currentCost;
    }
}

template <size_t N, size_t M>
Cost Exchange<N, M>::evaluate(Route::Node *U,
                              Route::Node *V,
                              CostEvaluator const &costEvaluator)
{
    if (containsDepot(U, N) || overlap(U, V))
        return 0;

    if constexpr (M > 0)
        if (containsDepot(V, M))
            return 0;

    if constexpr (M == 0)  // special case where nothing in V is moved
    {
        if (U == n(V))
            return 0;

        return evalRelocateMove(U, V, costEvaluator);
    }
    else
    {
        if constexpr (N == M)  // symmetric, so only have to evaluate this once
            if (U->client() >= V->client())
                return 0;

        if (adjacent(U, V))
            return 0;

        return evalSwapMove(U, V, costEvaluator);
    }
}

template <size_t N, size_t M>
void Exchange<N, M>::apply(Route::Node *U, Route::Node *V) const
{
    auto &uRoute = *U->route();
    auto &vRoute = *V->route();
    auto *uToInsert = N == 1 ? U : uRoute[U->idx() + N - 1];
    auto *insertUAfter = M == 0 ? V : vRoute[V->idx() + M - 1];

    // Insert these 'extra' nodes of U after the end of V...
    for (size_t count = 0; count != N - M; ++count)
    {
        auto *prev = p(uToInsert);
        uRoute.remove(uToInsert->idx());
        vRoute.insert(insertUAfter->idx() + 1, uToInsert);
        uToInsert = prev;
    }

    // ...and swap the overlapping nodes!
    for (size_t count = 0; count != M; ++count)
    {
        Route::swap(U, V);
        U = n(U);
        V = n(V);
    }
}
}  // namespace pyvrp::search

#endif  // PYVRP_EXCHANGE_H
