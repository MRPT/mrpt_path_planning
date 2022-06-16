/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/bits_math.h>  // 0.0_deg
#include <mrpt/poses/CPose2DGridTemplate.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <selfdriving/algos/CostEvaluator.h>
#include <selfdriving/algos/Planner.h>
#include <selfdriving/data/MotionPrimitivesTree.h>

#include <limits>
#include <unordered_map>

namespace selfdriving
{
struct TPS_Astar_Parameters
{
    TPS_Astar_Parameters() = default;
    static TPS_Astar_Parameters FromYAML(const mrpt::containers::yaml& c);

    double grid_resolution_xy  = 0.20;
    double grid_resolution_yaw = mrpt::DEG2RAD(5.0);

    double SE2_metricAngleWeight = 1.0;

    double heuristic_heading_weight = 0.1;  //!< [0,1]

    uint32_t max_ptg_trajectories_to_explore        = 20;
    double   ptg_norm_distance_sampling_granularity = 0.2;

    /** Required to smooth interpolation of rendered paths, evaluation of
     * path cost, etc. */
    size_t pathInterpolatedSegments = 5;

    /** 0:disabled */
    size_t saveDebugVisualizationDecimation = 0;

    mrpt::containers::yaml as_yaml();
    void                   load_from_yaml(const mrpt::containers::yaml& c);
};

using astar_heuristic_t = std::function<double(
    const SE2_KinState& /*from*/, const SE2_KinState& /*goal*/)>;

/**
 * Uses a SE(2) lattice to run an A* algorithm to find a kinematicaly feasible
 * path from "A" to "B" using a set of trajectories in the form of PTGs.
 *
 */
class TPS_Astar : virtual public mrpt::system::COutputLogger, public Planner
{
    DEFINE_MRPT_OBJECT(TPS_Astar, selfdriving)

   public:
    TPS_Astar();
    virtual ~TPS_Astar() = default;

    TPS_Astar_Parameters params_;

    PlannerOutput plan(const PlannerInput& in) override;

    mrpt::containers::yaml params_as_yaml() override
    {
        return params_.as_yaml();
    }

    void params_from_yaml(const mrpt::containers::yaml& c) override
    {
        params_.load_from_yaml(c);
    }

    distance_t default_heuristic(
        const SE2_KinState& from, const SE2_KinState& goal) const;

    astar_heuristic_t heuristic = astar_heuristic_t(
        [this](const SE2_KinState& from, const SE2_KinState& goal) {
            return this->default_heuristic(from, goal);
        });

   private:
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

        /** Integer cell indices for (x,y) in `grid_` */
        int32_t idxX = 0, idxY = 0;

        /** Phi or Yaw index in `grid_`, or none if undefined/arbitrary */
        std::optional<int32_t> idxYaw;
    };

    struct NodeCoordsHash
    {
        size_t operator()(const NodeCoords& x) const
        {
            size_t res = 17;
            res        = res * 31 + std::hash<int32_t>()(x.idxX);
            res        = res * 31 + std::hash<int32_t>()(x.idxY);
            if (x.idxYaw)
                res = res * 31 + std::hash<int32_t>()(x.idxYaw.value());
            return res;
        }
    };

    using nodes_with_exact_coordinates_t =
        std::unordered_map<NodeCoords, SE2_KinState, NodeCoordsHash>;

    using nodes_with_desired_speed_t =
        std::unordered_map<NodeCoords, relative_speed_t, NodeCoordsHash>;

    using absolute_cell_index_t = size_t;

    absolute_cell_index_t nodeCoordsToAbsIndex(const NodeCoords& n) const
    {
        return grid_.getSizeX() * grid_.getSizeY() * n.idxYaw.value() +
               grid_.getSizeX() * n.idxY + n.idxX;
    }

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

    /** Each of the nodes in the SE(2) lattice grid */
    struct Node
    {
        Node()  = default;
        ~Node() = default;

        std::optional<mrpt::graphs::TNodeID> id;

        //!< exact pose and velocity (no binning here)
        SE2_KinState state;

        /// Total cost from initialState to this node (default=Inf)
        distance_t gScore = std::numeric_limits<distance_t>::max();

        /// Guess of cost from this node to goal (default=Inf)
        distance_t fScore = std::numeric_limits<distance_t>::max();

        /// parent (precedent) of this node in the path.
        std::optional<const Node*> cameFrom;

        bool pendingInOpenSet = false;
        bool visited          = false;
    };

    mrpt::poses::CPose2DGridTemplate<Node> grid_;

    /// throws on out of grid limits.
    /// Returns a ref to the node.
    Node& getOrCreateNodeByPose(
        const selfdriving::SE2_KinState& p, MotionPrimitivesTreeSE2& tree,
        mrpt::graphs::TNodeID& nextFreeId)
    {
        Node& n = *grid_.getByPos(p.pose.x, p.pose.y, p.pose.phi);
        if (!n.id.has_value())
        {
            n.id    = nextFreeId++;
            n.state = p;
        }

        return n;
    }

    /// throws on out of grid limits.
    NodeCoords nodeGridCoords(const mrpt::math::TPose2D& p) const
    {
        return NodeCoords(
            grid_.x2idx(p.x), grid_.y2idx(p.y), grid_.phi2idx(p.phi));
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
        std::optional<ptg_index_t>        ptgIndex;
        std::optional<trajectory_index_t> ptgTrajIndex;
        std::optional<uint32_t>           relTrgStep;  //!< traj step index
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
        const SE2_KinState&                             goalState,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                                MAX_XY_OBSTACLES_CLIPPING_DIST,
        const nodes_with_exact_coordinates_t& nodesWithExactCoords,
        const nodes_with_desired_speed_t&     nodesWithSpeed);

    mrpt::maps::CPointsMap::Ptr cached_local_obstacles(
        const mrpt::math::TPose2D&                      queryPose,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                                          MAX_PTG_XY_DIST);
};

}  // namespace selfdriving
