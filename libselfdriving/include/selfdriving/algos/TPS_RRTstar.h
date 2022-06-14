/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/system/COutputLogger.h>
#include <selfdriving/algos/CostEvaluator.h>
#include <selfdriving/algos/Planner.h>

namespace selfdriving
{
struct TPS_RRTstar_Parameters
{
    TPS_RRTstar_Parameters() = default;
    static TPS_RRTstar_Parameters FromYAML(const mrpt::containers::yaml& c);

    double initialSearchRadius = 4.0;  //!< [m]
    double minStepLength       = 0.20;  //!< Between waypoints [m]
    double maxStepLength       = 1.00;  //!< Between waypoints [m]
    size_t maxIterations       = 10000;

    bool   drawInTPS           = true;  //!< Draw samples in TPS vs Euclidean
    double drawBiasTowardsGoal = 0.1;

    double headingToleranceGenerate = mrpt::DEG2RAD(90.0);
    double headingToleranceMetric   = mrpt::DEG2RAD(2.0);
    double metricDistanceEpsilon    = 0.01;

    double SE2_metricAngleWeight = 1.0;

    /** Required to smooth interpolation of rendered paths, evaluation of
     * path cost, etc. */
    size_t pathInterpolatedSegments = 5;

    /** 0:disabled */
    size_t saveDebugVisualizationDecimation = 0;

    mrpt::containers::yaml as_yaml();
    void                   load_from_yaml(const mrpt::containers::yaml& c);
};

class TPS_RRTstar : virtual public mrpt::system::COutputLogger, public Planner
{
    DEFINE_MRPT_OBJECT(TPS_RRTstar, selfdriving)

   public:
    TPS_RRTstar();
    virtual ~TPS_RRTstar() = default;

    TPS_RRTstar_Parameters params_;

    PlannerOutput plan(const PlannerInput& in) override;

    mrpt::containers::yaml params_as_yaml() override
    {
        return params_.as_yaml();
    }

    void params_from_yaml(const mrpt::containers::yaml& c) override
    {
        params_.load_from_yaml(c);
    }

   private:
    struct DrawFreePoseParams
    {
        DrawFreePoseParams(
            const PlannerInput& pi, const MotionPrimitivesTreeSE2& tree,
            const distance_t& searchRadius, const TNodeID goalNodeId)
            : pi_(pi),
              tree_(tree),
              searchRadius_(searchRadius),
              goalNodeId_(goalNodeId)
        {
        }

        const PlannerInput&            pi_;
        const MotionPrimitivesTreeSE2& tree_;
        const distance_t&              searchRadius_;
        const TNodeID                  goalNodeId_;
    };

    using closest_lie_nodes_list_t = std::map<
        distance_t,
        std::reference_wrapper<const MotionPrimitivesTreeSE2::node_t>>;

    using already_existing_node_t = std::optional<TNodeID>;

    using draw_pose_return_t = std::tuple<
        mrpt::math::TPose2D, already_existing_node_t, closest_lie_nodes_list_t>;

    draw_pose_return_t draw_random_free_pose(const DrawFreePoseParams& p);
    draw_pose_return_t draw_random_tps(const DrawFreePoseParams& p);
    draw_pose_return_t draw_random_euclidean(const DrawFreePoseParams& p);

    using path_to_nodes_list_t = std::map<
        distance_t,
        std::tuple<TNodeID, ptg_index_t, trajectory_index_t, distance_t>>;

    /** Find all existing nodes "x" in the tree **from which** we can reach
     * `query` (i.e. `other nodes` ==> `query`), and the motion primitives for
     * such motions. Note that, at this point, `query` does not have a velocity
     * state: it will be determined by the best motion primitive.
     *
     * This method is used in the EXTEND stage of RRT*, and uses the TP-Space
     * motion primitives (PTGs).
     *
     * \sa find_reachable_nodes_from(), find_nearby_nodes()
     */
    path_to_nodes_list_t find_source_nodes_towards(
        const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
        const double maxDistance, const TrajectoriesAndRobotShape& trs,
        const TNodeID                   goalNodeToIgnore,
        const closest_lie_nodes_list_t& hintCloseNodes);

    /** Find all existing nodes "x" in the tree within a given ball, given by
     * the metric on the Lie group, i.e. *not* following any particular PTG
     * trajectory.
     *
     * \sa find_reachable_nodes_from(), find_source_nodes_towards()
     */
    closest_lie_nodes_list_t find_nearby_nodes(
        const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
        const double maxDistance);

    std::tuple<distance_t, TNodeID> find_closest_node(
        const MotionPrimitivesTreeSE2& tree,
        const mrpt::math::TPose2D&     query) const;

    /** Find all existing nodes "x" in the tree that are **reachable from**
     * `query` (i.e. `query` ==> `other nodes`), and the motion primitives for
     * such motions. Here, `query` does includes a velocity state.
     *
     * This method is used in the REWIRE stage of RRT*.
     *
     * \sa find_source_nodes_towards()
     */
    path_to_nodes_list_t find_reachable_nodes_from(
        const MotionPrimitivesTreeSE2& tree, const TNodeID queryNodeId,
        const double maxDistance, const TrajectoriesAndRobotShape& trs,
        const closest_lie_nodes_list_t& hintCloseNodes,
        const std::optional<TNodeID>&   nodeToIgnoreHeading = std::nullopt);

    /** Returns local obstacles as seen from a given pose, clipped to a maximum
     * distance. */
    static void transform_pc_square_clipping(
        const mrpt::maps::CPointsMap& inMap,
        const mrpt::poses::CPose2D& asSeenFrom, const double MAX_DIST_XY,
        mrpt::maps::CPointsMap& outMap, bool appendToOutMap = true);

    /** Returns TPS-distance to obstacles.
     * ptg dynamic state must be updated by the caller.
     */
    distance_t tp_obstacles_single_path(
        const trajectory_index_t      tp_space_k_direction,
        const mrpt::maps::CPointsMap& localObstacles, const ptg_t& ptg);

    mrpt::maps::CPointsMap::Ptr cached_local_obstacles(
        const MotionPrimitivesTreeSE2& tree, const TNodeID nodeID,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                                          MAX_XY_DIST);

    /** for use in cached_local_obstacles(), local_obstacles_cache_ */
    struct LocalObstaclesInfo
    {
        mrpt::maps::CPointsMap::Ptr obs;
        mrpt::math::TPose2D         globalNodePose;
    };

    std::map<TNodeID, LocalObstaclesInfo> local_obstacles_cache_;
};

}  // namespace selfdriving
