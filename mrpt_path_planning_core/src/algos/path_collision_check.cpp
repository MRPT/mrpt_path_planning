/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/path_collision_check.h>
#include <mpp/algos/tp_obstacles_single_path.h>
#include <mpp/algos/transform_pc_square_clipping.h>
#include <mpp/ptgs/DiffDriveCollisionGridBased.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/maps/CSimplePointsMap.h>

namespace
{
// Distance from a pose beyond which obstacles cannot affect its PTG edges:
double obstacleReach(const mpp::ptg_t& ptg)
{
    if (const auto* g =
            dynamic_cast<const mpp::ptg::DiffDriveCollisionGridBased*>(&ptg);
        g != nullptr)
    {
        return g->getObstacleReachDistance();
    }
    return ptg.getRefDistance() + ptg.getMaxRobotRadius();
}
}  // namespace

std::optional<size_t> mpp::first_colliding_edge(
    const std::vector<const MoveEdgeSE2_TPS*>&      edges,
    const TrajectoriesAndRobotShape&                trs,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& obstacles)
{
    MRPT_START

    // Tolerance on the free distance along the edge [pseudometers]:
    constexpr double kTolerance = 1e-3;

    mrpt::maps::CSimplePointsMap localObs;
    for (size_t i = 0; i < edges.size(); i++)
    {
        const auto* e = edges[i];
        if (e == nullptr) { continue; }
        ASSERT_(e->ptgIndex >= 0);
        auto& ptg = *trs.ptgs.at(static_cast<size_t>(e->ptgIndex));

        ptg.updateNavDynamicState(e->getPTGDynState());
        if (auto* trim = dynamic_cast<ptg::SpeedTrimmablePTG*>(&ptg);
            trim != nullptr)
        {
            trim->trimmableSpeed_ = e->ptgTrimmableSpeed;
        }

        const double reach = obstacleReach(ptg);
        localObs.clear();
        for (const auto& obs : obstacles)
        {
            if (!obs) { continue; }
            transform_pc_square_clipping(
                *obs, mrpt::poses::CPose2D(e->stateFrom.pose), reach, localObs);
        }
        if (localObs.empty()) { continue; }

        const distance_t freeDist =
            tp_obstacles_single_path(e->ptgPathIndex, localObs, ptg);
        if (freeDist + kTolerance < e->ptgDist) { return i; }
    }
    return std::nullopt;

    MRPT_END
}
