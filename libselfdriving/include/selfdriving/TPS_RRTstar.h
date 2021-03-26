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
        bool   drawInTPS           = true;
        double minStepLength       = 0.15;  //!< Between waypoints [m]
        double maxStepLength       = 3.0;  //!< Between waypoints [m]
        size_t maxIterations       = 100000;
    };

    Parameters params_;

    /** Time profiler (Default: enabled)*/
    mrpt::system::CTimeLogger profiler_{true, "TPS_RRTstar"};

   private:
    struct DrawFreePoseParams
    {
        DrawFreePoseParams(
            const PlannerInput& pi, const MotionPrimitivesTreeSE2& tree)
            : pi_(pi), tree_(tree)
        {
        }

        const PlannerInput&            pi_;
        const MotionPrimitivesTreeSE2& tree_;
    };

    mrpt::math::TPose2D draw_random_free_pose(const DrawFreePoseParams& p);

    mrpt::math::TPose2D draw_random_tps(const DrawFreePoseParams& p);
    mrpt::math::TPose2D draw_random_euclidean(const DrawFreePoseParams& p);

    using closest_nodes_list_t = std::map<
        distance_t,
        std::tuple<TNodeID, ptg_index_t, trajectory_index_t, distance_t>>;

    closest_nodes_list_t find_nodes_within_ball(
        const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
        const double maxDistance, const TrajectoriesAndRobotShape& trs);

    /** Returns local obstacles as seen from a given pose, clipped to a maximum
     * distance. */
    static void transform_pc_square_clipping(
        const mrpt::maps::CPointsMap& inMap,
        const mrpt::poses::CPose2D& asSeenFrom, const double MAX_DIST_XY,
        mrpt::maps::CPointsMap& outMap);

    /** Returns TPS-distance to obstacles.
     * ptg dynamic state must be updated by the caller.
     */
    distance_t tp_obstacles_single_path(
        const trajectory_index_t      tp_space_k_direction,
        const mrpt::maps::CPointsMap& localObstacles, const ptg_t& ptg);

    mrpt::maps::CPointsMap::Ptr cached_local_obstacles(
        const MotionPrimitivesTreeSE2& tree, const TNodeID nodeID,
        const mrpt::maps::CPointsMap& globalObstacles, double MAX_XY_DIST);

    /** for use in cached_local_obstacles(), local_obstacles_cache_ */
    struct LocalObstaclesInfo
    {
        mrpt::maps::CPointsMap::Ptr obs;
        mrpt::math::TPose2D         globalNodePose;
    };

    std::map<TNodeID, LocalObstaclesInfo> local_obstacles_cache_;

    cost_t cost_path_segment(const MoveEdgeSE2_TPS& edge) const;
};

}  // namespace selfdriving
