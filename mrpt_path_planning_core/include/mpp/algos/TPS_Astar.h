/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/algos/Planner.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mrpt/core/bits_math.h>  // 0.0_deg
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <limits>
#include <unordered_map>

namespace mpp
{
using astar_heuristic_t = std::function<cost_t(
    const SE2_KinState& /*from*/, const SE2orR2_KinState& /*goal*/)>;

struct TPS_Astar_Parameters
{
    TPS_Astar_Parameters() = default;
    static TPS_Astar_Parameters FromYAML(const mrpt::containers::yaml& c);

    double grid_resolution_xy = 0.20;

    /** Default 7.5 deg (48 yaw bins). A finer 5 deg was found to over-resolve
     * the yaw dimension: since the heuristic is essentially positional, finer
     * yaw multiplies SE(2) expansions ~linearly with no quality gain. Measured
     * (collision-sound, 300-world BARN + 40-case HouseExpo): 5->7.5 deg cuts
     * mean plan time ~23-37% and HouseExpo median ~1.5x while *improving*
     * success (BARN 291->292/300, HouseExpo 39->40/40) at unchanged median path
     * quality. 10 deg is faster still but drops net BARN success, so 7.5 deg is
     * the sweet spot. See DESIGN.md sec 12.7. */
    double grid_resolution_yaw = 7.5_deg;

    double SE2_metricAngleWeight = 1.0;

    double heuristic_heading_weight = 0.1;  //!< [0,1]

    /** Weighted-A* heuristic inflation factor `eps >= 1` (default 1.0 = exact
     * A*). The open set is ordered by `f = g + eps * h`. Values `> 1` make the
     * search greedier: it expands far fewer nodes and returns a solution whose
     * cost is provably within a factor `eps` of the optimum (bounded
     * sub-optimality, as in ARA* / SBPL). Since the planner's expensive worlds
     * are dominated by expansion count, `eps` in `[1.5, 2]` is the cheapest
     * lever to cut the worst-case latency tail at a small, bounded path-cost
     * increase. Found to be 1.5 to be somehow better in runtime than 1.0 in
     * general cases. */
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

    TPS_Astar_Parameters params_;

    PlannerOutput plan(const PlannerInput& in) override;

    mrpt::containers::yaml params_as_yaml() override
    { return params_.as_yaml(); }

    void params_from_yaml(const mrpt::containers::yaml& c) override
    { params_.load_from_yaml(c); }

    cost_t default_heuristic(
        const SE2_KinState& from, const SE2orR2_KinState& goal) const;

    cost_t default_heuristic_SE2(
        const SE2_KinState& from, const mrpt::math::TPose2D& goal) const;
    cost_t default_heuristic_R2(
        const SE2_KinState& from, const mrpt::math::TPoint2D& goal) const;

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
        { seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2); }

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
    { return static_cast<int32_t>(std::round(x / params_.grid_resolution_xy)); }
    int32_t y2idx(float y) const
    { return static_cast<int32_t>(std::round(y / params_.grid_resolution_xy)); }
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
    { return NodeCoords(x2idx(p.x), y2idx(p.y), phi2idx(p.phi)); }
    NodeCoords nodeGridCoords(const mrpt::math::TPoint2D& p) const
    { return NodeCoords(x2idx(p.x), y2idx(p.y)); }

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

    /** Maximum linear speed across all active PTGs, cached at the start of
     *  each plan() call. Used to convert geometric distances (meters) to
     *  time estimates (seconds) in the heuristic, ensuring admissibility when
     *  edge costs are in seconds (estimatedExecTime). Defaults to 1.0 so
     *  that heuristic calls outside plan() return geometric distances. */
    double maxLinSpeed_ = 1.0;

    /** Cache of local obstacle maps, keyed by (ix, iy) grid cell (no yaw,
     *  since obstacle clipping only depends on xy position). Cleared at the
     *  start of each plan() call. Nodes in the same cell share the same
     *  transformed obstacle cloud, avoiding redundant O(N_obs) transforms. */
    std::unordered_map<NodeCoords, mrpt::maps::CPointsMap::Ptr, NodeCoordsHash>
        localObstaclesCache_;
};

}  // namespace mpp
