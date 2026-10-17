/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/algos/Planner.h>
#include <mpp/algos/reeds_shepp.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mrpt/core/bits_math.h>  // 0.0_deg
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>

/** Feature macro: TPS_Astar has the resumable search API (start(), run(),
 * reroot(), set_obstacles()). */
#define MPP_HAS_RESUMABLE_ASTAR 1

namespace mpp
{
using astar_heuristic_t = std::function<cost_t(
    const SE2_KinState& /*from*/, const SE2orR2_KinState& /*goal*/)>;

struct TPS_Astar_Parameters
{
    TPS_Astar_Parameters() = default;
    static TPS_Astar_Parameters FromYAML(const mrpt::containers::yaml& c);

    double grid_resolution_xy = 0.20;

    /** Default 7.5 deg (48 yaw bins). Since the heuristic is essentially
     * positional, a finer yaw resolution multiplies the expansions with no
     * quality gain, while a coarser one loses success rate. */
    double grid_resolution_yaw = 7.5_deg;

    double SE2_metricAngleWeight = 1.0;

    double heuristic_heading_weight = 0.1;  //!< [0,1]

    /** Weighted-A* heuristic inflation factor `eps >= 1` (1.0 = exact
     * A*, default 1.5). The open set is ordered by `f = g + eps * h`. Values `>
     * 1` make the search greedier: it expands far fewer nodes and returns a
     * solution whose cost is provably within a factor `eps` of the optimum
     * (bounded sub-optimality, as in ARA* / SBPL). Since the planner's
     * expensive worlds are dominated by expansion count, `eps` in `[1.5, 2]` is
     * the cheapest lever to cut the worst-case latency tail at a small, bounded
     * path-cost increase. Found to be 1.5 to be somehow better in runtime
     * than 1.0 in general cases. */
    double heuristic_epsilon = 1.5;

    /** Analytic expansion / early termination:
     * `find_feasible_paths_to_neighbors` already builds a collision-free
     * *direct-to-goal* PTG edge (via the PTG inverse map) whenever the goal is
     * within reach of the node being expanded. When this is enabled (default),
     * the search terminates the moment such a connection lands in the goal
     * cell, instead of continuing A* until the goal node is popped as the
     * lowest-f node. This is the classic analytic-expansion speedup (cf.
     * Hybrid-A* / Nav2 Smac): it skips the costly final-approach expansions at
     * the price of accepting the first proven connection to the goal (a small,
     * bounded optimality relaxation). Set false for strictly optimal (slower)
     * termination. */
    bool use_analytic_expansion = true;

    /** For full-pose (SE(2)) goals, use the Reeds-Shepp path length divided by
     * the maximum linear speed as heuristic. It is a lower bound of the travel
     * time of any path made of forward and reverse circular arcs with radius
     * no smaller than the minimum turning radius, hence admissible and
     * consistent, and it accounts for the maneuvering (e.g., reversals)
     * needed to reach a goal heading, as in Hybrid-A*. Only applied if all
     * PTGs are of type DiffDrive_C (the minimum turning radius is
     * v_max/w_max); otherwise the default heuristic is used. */
    bool use_reeds_shepp_heuristic = false;

    /** Try a Reeds-Shepp shot from expanded nodes within
     * `reeds_shepp_expansion_max_length` [m] of the goal: a Reeds-Shepp path
     * at the tightest turning radius of the PTGs, executed exactly as a chain
     * of edges along the tightest-arc and straight trajectories of forward
     * and reverse C-PTGs. Every edge is checked with the same certified
     * TP-obstacle query as any other edge. A collision-free shot becomes a
     * goal candidate of the deferred analytic expansion (so
     * `use_analytic_expansion` must be enabled), which keeps the
     * suboptimality bound, and it ends at the goal itself, not only in the
     * goal cell.
     *
     * From every expanded node: the shortest path to the goal pose
     * (full-pose goals) or to the goal point with a free final heading
     * (position goals, see reeds_shepp_path_to_point()).
     *
     * A search that would end in the goal cell first tries a shot from there
     * and, if it is clear, goes on until a shot is committed. For a position
     * goal, it then also tries the shortest paths ending at every other final
     * heading (sampled every `grid_resolution_yaw`) at which the footprint
     * fits at the goal, from that node and from its ancestors within reach,
     * since a goal close to obstacles may only admit some headings, reached
     * from farther away. If the footprint fits at no heading, no shots are
     * tried for a position goal. It requires
     * forward and reverse DiffDrive_C PTGs with an odd number of trajectories
     * (so that one is straight); otherwise it is silently not used. */
    bool   use_reeds_shepp_expansion        = true;
    double reeds_shepp_expansion_max_length = 5.0;

    uint32_t                        max_ptg_trajectories_to_explore = 20;
    std::vector<duration_seconds_t> ptg_sample_timestamps     = {1.0, 3.0, 5.0};
    uint32_t                        max_ptg_speeds_to_explore = 3;

    /** Required to smooth interpolation of rendered paths, evaluation of
     * path cost, etc.
     * Even if this is 0, the interpolated path container `interpolatedPath`
     * will contain the start and final pose, along with their times.
     */
    size_t pathInterpolatedSegments = 5;

    /** 0:disabled */
    size_t saveDebugVisualizationDecimation = 0;
    bool   debugVisualizationShowEdgeCosts  = false;

    /** Maximum time to spend looking for a solution */
    duration_seconds_t maximumComputationTime =
        std::numeric_limits<duration_seconds_t>::max();

    mrpt::containers::yaml as_yaml();
    void                   load_from_yaml(const mrpt::containers::yaml& c);
};

/**
 * Uses a SE(2) lattice to run an A* algorithm to find a kinematically feasible
 * path from "A" to "B" using a set of trajectories in the form of PTGs.
 *
 * ## Cost model and optimality
 *
 * This planner optimizes **SE(2) path cost**, not R(2) path length. The base
 * edge cost is `estimatedExecTime` (travel time), and the default heuristic
 * combines a Lie-group distance with a heading-alignment penalty:
 *
 *   h = dist_SE2(from, goal) + w * |angDistance(atan2(dy,dx), phi)|
 *
 * The heading term is intentional and physically meaningful: for any vehicle
 * capable of rotation (holonomic, differential-drive, Ackermann), the true
 * cost-to-go in SE(2) includes the effort to reorient. A path that arrives at
 * the goal position facing the wrong direction is genuinely more expensive than
 * one that arrives correctly aligned. The planner therefore finds paths that
 * are optimal in SE(2), which are not necessarily the shortest in R(2).
 *
 * To minimize only R(2) path length (ignoring final heading cost), set:
 *   - `SE2_metricAngleWeight = 0`
 *   - `heuristic_heading_weight = 0`
 */
class TPS_Astar : virtual public mrpt::system::COutputLogger, public Planner
{
    DEFINE_MRPT_OBJECT(TPS_Astar, mpp)

   public:
    TPS_Astar();
    virtual ~TPS_Astar() = default;

    /** Copies the configuration (parameters, cost evaluators, logger) but not
     * the search state, which points into the lattice of the source. The copy
     * uses the default heuristic bound to itself. */
    TPS_Astar(const TPS_Astar& o);
    TPS_Astar& operator=(const TPS_Astar& o);

    TPS_Astar_Parameters params_;

    PlannerOutput plan(const PlannerInput& in) override;

    /** @name Resumable search
     * plan() is start() followed by one run(). Calling them separately lets
     * a caller search in time slices, use the best path found so far before
     * the search ends (anytime planning), move the root of the search along
     * the path being executed (reroot()), and update the obstacles for the
     * rest of the search (set_obstacles()).
     * Feature macro: MPP_HAS_RESUMABLE_ASTAR.
     * @{ */

    /** Starts a new search (expands no node yet). */
    void start(const PlannerInput& in);

    /** Continues the search started with start() for up to
     * `maxComputationTime` [s], or until it ends. \return The current
     * result: success if the goal was reached, otherwise `bestNodeId` is the
     * node closest to the goal (by the heuristic) found so far. Edges along
     * the path to `bestNodeId` are interpolated. */
    PlannerOutput run(duration_seconds_t maxComputationTime);

    /** Light result of a search slice (see run_slice()). */
    struct SliceResult
    {
        bool   success          = false;
        double computationTime  = 0;  //!< total [s]
        size_t numExpandedNodes = 0;
        double pathCost         = std::numeric_limits<double>::max();
        std::optional<mrpt::graphs::TNodeID> goalNodeId;
        std::optional<mrpt::graphs::TNodeID> bestNodeId;
        cost_t bestNodeIdCostToGoal = std::numeric_limits<cost_t>::max();
    };

    /** Like run(), but without copying the motion tree: returns only ids and
     * costs. Use path_to_node() to get the path. */
    SliceResult run_slice(duration_seconds_t maxComputationTime);

    /** Nodes and edges of the path from the search root to `nodeId` (e.g.
     * `SliceResult::bestNodeId`), with the edges interpolated. They point into
     * the search tree: valid until the next run(), reroot() or start(). */
    std::tuple<
        MotionPrimitivesTreeSE2::path_t,
        MotionPrimitivesTreeSE2::edge_sequence_t>
        path_to_node(mrpt::graphs::TNodeID nodeId);

    /** Whether start() was called and the search can go on (the goal was
     * not reached yet, and the open set is not empty). */
    [[nodiscard]] bool search_can_continue() const;

    /** Whether the search reached the goal. */
    [[nodiscard]] bool search_reached_goal() const { return search_.found; }

    /** Makes the tree node `newRoot` the new start of the search: only its
     * subtree is kept (lattice cells of the other nodes are freed, so they
     * can be reached again from it), costs are relative to it, and the
     * search goes on from the open nodes of the subtree. The path from the
     * old root to `newRoot` is thus committed. Visited nodes of the subtree
     * are not expanded again, so freed cells only become reachable from its
     * open nodes.
     * \return false if `newRoot` is not in the tree (nothing is changed). */
    bool reroot(mrpt::graphs::TNodeID newRoot);

    /** Replaces the obstacles for the rest of the search. Edges already in
     * the tree are not checked again: callers must validate the path they
     * use against the new obstacles. */
    void set_obstacles(const std::vector<ObstacleSource::Ptr>& obstacles);

    /** @} */

    mrpt::containers::yaml params_as_yaml() override
    {
        return params_.as_yaml();
    }

    void params_from_yaml(const mrpt::containers::yaml& c) override
    {
        params_.load_from_yaml(c);
    }

    cost_t default_heuristic(
        const SE2_KinState& from, const SE2orR2_KinState& goal) const;

    cost_t default_heuristic_SE2(
        const SE2_KinState& from, const mrpt::math::TPose2D& goal) const;
    cost_t default_heuristic_R2(
        const SE2_KinState& from, const mrpt::math::TPoint2D& goal) const;

    /** Minimum turning radius of a set of DiffDrive_C PTGs (the Reeds-Shepp
     * heuristic radius), or 0 if any PTG is of another type. */
    static double reeds_shepp_turning_radius(
        const TrajectoriesAndRobotShape& trs);

    astar_heuristic_t heuristic = astar_heuristic_t(
        [this](const SE2_KinState& from, const SE2orR2_KinState& goal)
        { return this->default_heuristic(from, goal); });

    // NOTE: members below are `protected` (not `private`) so that the
    // bidirectional variant `TPS_Astar_Bidir` can reuse the lattice/binning,
    // the neighbor generation, the collision machinery (cached_local_obstacles)
    // and the heuristics without forking any of the collision/PTG code.
   protected:
    struct NodeCoords
    {
        NodeCoords() = default;

        NodeCoords(int32_t ix, int32_t iy) : idxX(ix), idxY(iy) {}
        NodeCoords(int32_t ix, int32_t iy, int32_t iphi)
            : idxX(ix), idxY(iy), idxYaw(iphi)
        {
        }

        NodeCoords operator+(const NodeCoords& o) const
        {
            return {
                idxX + o.idxX, idxY + o.idxY,
                idxYaw.value() + o.idxYaw.value()};
        }

        std::string asString() const
        {
            std::string r = "(";
            r += std::to_string(idxX);
            r += ",";
            r += std::to_string(idxY);
            if (idxYaw.has_value())
            {
                r += ",";
                r += std::to_string(*idxYaw);
            }
            r += ")";
            return r;
        }

        bool operator==(const NodeCoords& o) const
        {
            if (idxX != o.idxX || idxY != o.idxY) return false;
            if (idxYaw.has_value() != o.idxYaw.has_value()) return false;
            if (idxYaw.has_value() && idxYaw.value() != o.idxYaw.value())
                return false;
            return true;
        }
        bool operator!=(const NodeCoords& o) const { return !(*this == o); }

        /** Returns true if:
         *  (1) two cells have heading and all (x,y,phi) are the same; or
         *  (2) at least one cell has no heading defined, and (x,y) are equal.
         */
        bool sameLocation(const NodeCoords& o) const
        {
            if (idxX != o.idxX || idxY != o.idxY) return false;
            if (idxYaw.has_value() && o.idxYaw.has_value())
                return idxYaw.value() == o.idxYaw.value();
            // same (x,y), indifferent heading:
            return true;
        }

        /** Integer cell indices for (x,y) in `grid_` */
        int32_t idxX = 0, idxY = 0;

        /** Phi or Yaw index in `grid_`, or none if undefined/arbitrary */
        std::optional<int32_t> idxYaw;
    };

    struct NodeCoordsHash
    {
        // boost::hash_combine pattern: avalanches bits so that adjacent
        // integer grid coordinates map to well-separated hash buckets.
        static void hash_combine(size_t& seed, size_t v)
        {
            seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        size_t operator()(const NodeCoords& x) const
        {
            size_t res = 0;
            hash_combine(res, std::hash<int32_t>()(x.idxX));
            hash_combine(res, std::hash<int32_t>()(x.idxY));
            if (x.idxYaw)
                hash_combine(res, std::hash<int32_t>()(x.idxYaw.value()));
            return res;
        }
    };

#if 0
    using nodes_with_exact_coordinates_t =
        std::unordered_map<NodeCoords, SE2_KinState, NodeCoordsHash>;
#endif

    using nodes_with_desired_speed_t =
        std::unordered_map<NodeCoords, normalized_speed_t, NodeCoordsHash>;

#if 0
    mrpt::math::TPose2D nodeCoordsToPose(
        const NodeCoords&                     n,
        const nodes_with_exact_coordinates_t& specialNodes) const
    {
        if (const auto it = specialNodes.find(n); it != specialNodes.end())
            return it->second.pose;

        return {
            grid_.idx2x(n.idxX), grid_.idx2y(n.idxY),
            grid_.idx2phi(n.idxYaw.value())};
    }
#endif

    /** Each of the nodes in the SE(2) lattice grid */
    struct Node
    {
        Node()  = default;
        ~Node() = default;

        std::optional<mrpt::graphs::TNodeID> id;

        //!< exact pose and velocity (no binning here)
        SE2_KinState state;

        /// Total cost from initialState to this node (default=Inf)
        cost_t gScore = std::numeric_limits<cost_t>::max();

        /// Guess of cost from this node to goal (default=Inf)
        cost_t fScore = std::numeric_limits<cost_t>::max();

        /// parent (precedent) of this node in the path.
        std::optional<const Node*> cameFrom;

        bool pendingInOpenSet = false;
        bool visited          = false;
    };

    using SE2_Lattice = std::unordered_map<NodeCoords, Node, NodeCoordsHash>;

    SE2_Lattice grid_;

    int32_t x2idx(float x) const
    {
        return static_cast<int32_t>(std::round(x / params_.grid_resolution_xy));
    }
    int32_t y2idx(float y) const
    {
        return static_cast<int32_t>(std::round(y / params_.grid_resolution_xy));
    }
    int32_t phi2idx(float yaw) const
    {
        const auto phi = mrpt::math::wrapToPi(yaw);
        return static_cast<int32_t>(
            std::round(phi / params_.grid_resolution_yaw));
    }

    /// throws on out of grid limits.
    /// Returns a ref to the node.
    Node& getOrCreateNodeByPose(
        const mpp::SE2_KinState& p, mrpt::graphs::TNodeID& nextFreeId);

    /// throws on out of grid limits.
    NodeCoords nodeGridCoords(const mrpt::math::TPose2D& p) const
    {
        return NodeCoords(x2idx(p.x), y2idx(p.y), phi2idx(p.phi));
    }
    NodeCoords nodeGridCoords(const mrpt::math::TPoint2D& p) const
    {
        return NodeCoords(x2idx(p.x), y2idx(p.y));
    }

    struct NodePtr
    {
        NodePtr()  = default;
        ~NodePtr() = default;

        NodePtr(Node* p) : ptr(p) {}

        Node* operator->()
        {
            ASSERT_(ptr);
            return ptr;
        }
        const Node* operator->() const
        {
            ASSERT_(ptr);
            return ptr;
        }
        Node& operator*()
        {
            ASSERT_(ptr);
            return *ptr;
        }
        const Node& operator*() const
        {
            ASSERT_(ptr);
            return *ptr;
        }

        Node* ptr = nullptr;
    };

    struct path_to_neighbor_t
    {
        std::optional<ptg_index_t>             ptgIndex;
        std::optional<trajectory_index_t>      ptgTrajIndex;
        std::optional<uint32_t>                relTrgStep;  //!< traj step index
        std::optional<ptg_t::TNavDynamicState> ptgDynState;
        normalized_speed_t                     ptgTrimmableSpeed = 1.0;
        distance_t          ptgDist = std::numeric_limits<distance_t>::max();
        mrpt::math::TPose2D relReconstrPose;
        NodeCoords          neighborNodeCoords;
    };

    using list_paths_to_neighbors_t = std::vector<path_to_neighbor_t>;

    /** This generates a list of many potential neighbor cells to visit, subject
     * to kinematic and dynamic limitations, and obstacle checking.
     */
    list_paths_to_neighbors_t find_feasible_paths_to_neighbors(
        const Node& from, const TrajectoriesAndRobotShape& trs,
        const SE2orR2_KinState&                         goalState,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                            MAX_XY_OBSTACLES_CLIPPING_DIST,
        const nodes_with_desired_speed_t& nodesWithSpeed,
        const mrpt::math::TPose2D&        worldBboxMin,
        const mrpt::math::TPose2D&        worldBboxMax);

    mrpt::maps::CPointsMap::Ptr cached_local_obstacles(
        const mrpt::math::TPose2D&                      queryPose,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                                          MAX_PTG_XY_DIST);

    /** The global obstacles within the clipping window of the lattice xy cell
     * of `queryPose`, in the global frame, cached per cell. */
    mrpt::maps::CPointsMap::Ptr clipped_global_obstacles(
        const mrpt::math::TPose2D&                      queryPose,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                                          MAX_PTG_XY_DIST);

    /** As cached_local_obstacles(), but writes the local obstacles into the
     * reused buffers localObsX_, localObsY_ instead of a new points map.
     * \return The number of local obstacle points. */
    size_t local_obstacles_to_buffers(
        const mrpt::math::TPose2D&                      queryPose,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                                          MAX_PTG_XY_DIST);

    std::vector<float> localObsX_, localObsY_;

    /** Start pose of the current plan, and handling of a start closer than
     * the PTG clearance to some obstacle (e.g. a robot that stopped close to
     * a wall): the clearance-inflated collision grids would then block every
     * motion, so edges leaving the start are instead checked with an exact
     * footprint sweep that only forbids getting closer than
     * `startSweepThreshold_` (half the current distance, at most the
     * clearance) to any obstacle. If the start pose already touches some
     * obstacles (e.g. a robot that stopped in contact, or sensor noise),
     * those may not get any closer than they are (`startObstacleDist0_`),
     * so the robot can still move away from them. See
     * local_obstacles_to_buffers() and start_edge_is_clear(). */
    std::optional<mrpt::math::TPose2D> startPose_;
    RobotShape                         startRobotShape_;
    double                             startClearance_       = 0;
    bool                               startWithinClearance_ = false;
    double                             startSweepThreshold_  = 0;
    std::vector<mrpt::math::TPoint2D>  startLocalObstacles_;
    /** Signed distance of each startLocalObstacles_ point to the footprint
     * at the start pose, only if the start touches obstacles (else empty) */
    std::vector<double> startObstacleDist0_;

    /** Exact sweep check of an edge leaving the start pose (only used if
     * startWithinClearance_). */
    bool start_edge_is_clear(
        const ptg_t& ptg, trajectory_index_t k, uint32_t step) const;

    /** Free distance along every trajectory of each PTG (indexed as the PTGs)
     * from `expandedTpObstaclesPose_`, the pose of the last node expanded by
     * find_feasible_paths_to_neighbors(). Its Reeds-Shepp shots start there,
     * so they reuse them instead of repeating the query. */
    std::vector<std::vector<double>>   expandedTpObstacles_;
    std::optional<mrpt::math::TPose2D> expandedTpObstaclesPose_;

    /** Distance from a node beyond which obstacles cannot affect any edge of
     * this PTG: trajectory length plus footprint radius (and clearance, for
     * collision-grid PTGs). Used to clip local obstacles soundly. */
    static double obstacle_clipping_distance(const ptg_t& ptg);

    /** Maximum linear speed across all active PTGs, cached at the start of
     *  each plan() call. Used to convert geometric distances (meters) to
     *  time estimates (seconds) in the heuristic, ensuring admissibility when
     *  edge costs are in seconds (estimatedExecTime). Defaults to 1.0 so
     *  that heuristic calls outside plan() return geometric distances. */
    double maxLinSpeed_ = 1.0;

    /** Turning radius for the Reeds-Shepp heuristic [m], 0 = not applicable.
     * Set in plan() from the PTGs. */
    double reedsSheppTurningRadius_ = 0.0;

    /** Trajectories of one C-PTG used by the Reeds-Shepp shot. */
    struct RsExpansionPtg
    {
        size_t             ptgIndex  = 0;
        trajectory_index_t kLeft     = 0;
        trajectory_index_t kRight    = 0;
        trajectory_index_t kStraight = 0;
        bool               valid     = false;
    };
    RsExpansionPtg rsForward_, rsReverse_;
    /** Tightest turning radius of the PTGs [m], 0 = shot not available. */
    double rsExpansionRadius_ = 0.0;

    /** Sets rsForward_, rsReverse_ and rsExpansionRadius_ from the PTGs.
     * \return false if they do not support the Reeds-Shepp shot. */
    bool prepare_reeds_shepp_expansion(const TrajectoriesAndRobotShape& trs);

    /** For a position goal, the obstacle points near it, relative to the
     * goal point, the final headings at which the footprint, grown by the
     * certified clearance of the shot PTGs, fits at the goal, and that
     * clearance [m] (see prepare_point_goal_shots()). */
    std::vector<mrpt::math::TPoint2D> rsGoalObstacles_;
    std::vector<double>               rsGoalHeadings_;
    double                            rsGoalClearance_ = 0;

    /** Fills rsGoalObstacles_, rsGoalHeadings_ and rsGoalClearance_ for the
     * goal point. */
    void prepare_point_goal_shots(
        const mrpt::math::TPoint2D& goal, const TrajectoriesAndRobotShape& trs,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles);

    /** Whether the footprint, grown by the certified clearance, contains no
     * point of rsGoalObstacles_ with the heading `phi` at the goal point.
     * Shots ending at other headings cannot pass the certified check. */
    bool footprint_fits_at_goal(
        double phi, const TrajectoriesAndRobotShape& trs) const;

    /** One edge of a Reeds-Shepp shot, ready to be inserted in the tree. */
    struct RsShotEdge
    {
        MoveEdgeSE2_TPS edge;
        SE2_KinState    stateTo;
    };

    /** Builds the Reeds-Shepp shot from `from` to the goal pose or point as
     * certified PTG edges (see use_reeds_shepp_expansion), no longer than
     * `maxLength` [m]. For a position goal, the shortest path with a free
     * final heading is tried and, with `allHeadings`, then those to every
     * final heading at which the footprint fits. \return An empty vector if
     * there is none (too long, colliding, leaving the world box, or `from`
     * already at the goal point). */
    std::vector<RsShotEdge> reeds_shepp_shot(
        const Node& from, const SE2orR2_KinState& goal, double maxLength,
        bool allHeadings, const TrajectoriesAndRobotShape& trs,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double MAX_XY_OBSTACLES_CLIPPING_DIST, const PlannerInput& in);

    /** Executes the Reeds-Shepp `segments` from `from` as certified PTG
     * edges, which must end at the goal (see reeds_shepp_shot()). */
    std::vector<RsShotEdge> execute_shot(
        const Node& from, const std::vector<ReedsSheppSegment>& segments,
        const SE2orR2_KinState& goal, const TrajectoriesAndRobotShape& trs,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double MAX_XY_OBSTACLES_CLIPPING_DIST, const PlannerInput& in);

    /** Cache of local obstacle maps, keyed by (ix, iy) grid cell (no yaw,
     *  since obstacle clipping only depends on xy position). Cleared at the
     *  start of each plan() call. Nodes in the same cell share the same
     *  transformed obstacle cloud, avoiding redundant O(N_obs) transforms. */
    std::unordered_map<NodeCoords, mrpt::maps::CPointsMap::Ptr, NodeCoordsHash>
        localObstaclesCache_;

    /** Entry of the open set, ordered by fScore; ties are popped in insertion
     * order (FIFO). */
    struct OpenEntry
    {
        distance_t fScore;
        uint64_t   seq;
        Node*      node;
        bool       operator>(const OpenEntry& o) const
        {
            return fScore != o.fScore ? fScore > o.fScore : seq > o.seq;
        }
    };
    using open_set_t = std::priority_queue<
        OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>>;

    /** State of the search between start() and the end of the search, so it
     * can be resumed (see run()). */
    struct SearchState
    {
        bool started   = false;
        bool found     = false;  //!< goal reached
        bool exhausted = false;  //!< open set empty

        PlannerInput  in;
        PlannerOutput po;

        std::vector<mrpt::maps::CPointsMap::Ptr> obstaclePoints;
        double                                   maxXYDist = 0;

        open_set_t            openSet;
        uint64_t              openSeq    = 0;
        mrpt::graphs::TNodeID nextFreeId = 0;

        Node*                      nodeGoal = nullptr;
        NodeCoords                 goalCellIndices;
        nodes_with_desired_speed_t nodesWithDesiredSpeed;

        unsigned int nIter         = 0;
        double       tLastCallback = 0;

        /** Deferred analytic expansion: cheapest goal-landing candidate (see
         * plan()) */
        Node*  bestGoalCandidate  = nullptr;
        cost_t bestGoalCandidateG = std::numeric_limits<cost_t>::max();
        /** A goal candidate may also be the last node of a Reeds-Shepp shot,
         * which lives in the tree only (not in the lattice) */
        std::optional<mrpt::graphs::TNodeID> bestGoalCandidateShotId;

        bool deferInterpolation = false;
    };
    SearchState search_;

    /** Extracts the obstacle points of search_.in.obstacles, and prepares
     * everything depending on them. */
    void prepare_obstacles();

    /** Expands nodes for up to `maxComputationTime` [s] or until the search
     * ends. */
    void run_search(duration_seconds_t maxComputationTime);

    /** Completes a copy of the search output: goal shots not committed yet,
     * deferred edge interpolation, success flag, path cost. */
    PlannerOutput finalized_output(PlannerOutput po) const;

    /** Builds the deferred interpolated paths of the edges to `nodeId`. */
    void interpolate_path_to(
        MotionPrimitivesTreeSE2& tree, mrpt::graphs::TNodeID nodeId) const;

    /** Output ids and costs, with a not yet committed Reeds-Shepp shot. */
    SliceResult make_slice_result(const PlannerOutput& po) const;
};

}  // namespace mpp
