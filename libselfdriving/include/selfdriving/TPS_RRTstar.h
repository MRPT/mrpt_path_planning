/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <selfdriving/PlannerInput.h>
#include <selfdriving/PlannerOutput.h>

namespace selfdriving
{
class TPS_RRTstar : public mrpt::system::COutputLogger
{
   public:
    TPS_RRTstar();
    ~TPS_RRTstar() = default;

    PlannerOutput plan(const PlannerInput& in);

    struct Parameters
    {
        double goalBias            = 0.02;
        double initialSearchRadius = 1.0;
    };

    Parameters params_;

    /** Time profiler (Default: enabled)*/
    mrpt::system::CTimeLogger profiler_{true, "TPS_RRTstar"};

   private:
    struct DrawFreePoseParams
    {
        DrawFreePoseParams(const PlannerInput& pi) : pi_(pi) {}

        const PlannerInput& pi_;
    };

    mrpt::math::TPose2D draw_random_free_pose(const DrawFreePoseParams& p);

    using closest_nodes_list_t =
        std::map<distance_t, std::pair<TNodeID, trajectory_index_t>>;

    closest_nodes_list_t find_nodes_within_ball(
        const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
        const double maxDistance, const TrajectoriesAndRobotShape& trs);

    static void transform_pc_square_clipping(
        const mrpt::maps::CPointsMap& in_map, mrpt::maps::CPointsMap& out_map,
        const mrpt::poses::CPose2D& asSeenFrom, const double MAX_DIST_XY);

    /// Returns normalized TPS-distances to obstacles.
    std::vector<double> transform_to_tps(
        const mrpt::maps::CSimplePointsMap& in_obstacles, const ptg_t& ptg,
        const double MAX_DIST);

    /// Returns normalized TPS-distance to obstacles.
    double transform_to_tps_single_path(
        const int                           tp_space_k_direction,
        const mrpt::maps::CSimplePointsMap& in_obstacles, const ptg_t& ptg,
        const double MAX_DIST);
};

}  // namespace selfdriving
