/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/containers/traits_map.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/graphs/CDirectedTree.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/poses/CPose2D.h>
#include <selfdriving/MoveEdgeSE2_TPS.h>
#include <selfdriving/SE2_KinState.h>
#include <selfdriving/ptg_t.h>

#include <cstdint>
#include <list>
#include <set>

namespace selfdriving
{
/** Generic base for metrics */
template <class node_t>
struct PoseDistanceMetric;

/** Distances measured by PoseDistanceMetric */
using distance_t = double;

/** Index of a trajectory in a PTG */
using trajectory_index_t = int;

/** A tree with nodes being vehicle poses, and edges potential valid motion
 * primitives between them.
 *
 * This class provides storage for the nodes, and RRT* construction helper
 * methods.
 *
 * See base class mrpt::graphs::CDirectedTree for the API to access edges.
 *
 * *Changes history*:
 *  - 06/MAR/2014: Creation (MB)
 *  - 21/JAN/2015: Refactoring (JLBC)
 *  - 2020-2021: Adapted to TPS-RRT* (JLBC)
 */
template <
    class NODE_TYPE_DATA, class EDGE_TYPE,
    class MAPS_IMPLEMENTATION = mrpt::containers::map_traits_map_as_vector
    /* Use std::map<> vs. std::vector<>*/
    >
class MotionPrimitivesTree : public mrpt::graphs::CDirectedTree<EDGE_TYPE>
{
   public:
    struct node_t : public NODE_TYPE_DATA
    {
        /** Duplicated ID (it's also in the map::iterator->first), but put here
         * to make it available in path_t */
        mrpt::graphs::TNodeID nodeID_ = INVALID_NODEID;

        /** Does not have value for the root, a valid ID otherwise */
        std::optional<mrpt::graphs::TNodeID> parentID_;

        /** NULL for root, a valid edge otherwise */
        EDGE_TYPE* edgeToParent_ = nullptr;

        node_t() = default;
        node_t(
            mrpt::graphs::TNodeID                       nodeID,
            const std::optional<mrpt::graphs::TNodeID>& parentID,
            EDGE_TYPE* edgeToParent, const NODE_TYPE_DATA& data)
            : NODE_TYPE_DATA(data),
              nodeID_(nodeID),
              parentID_(parentID),
              edgeToParent_(edgeToParent)
        {
        }
    };

    using base_t = mrpt::graphs::CDirectedTree<EDGE_TYPE>;
    using edge_t = EDGE_TYPE;

    /** Map: TNode_ID => Node info */
    using node_map_t = typename MAPS_IMPLEMENTATION::template map<
        mrpt::graphs::TNodeID, node_t>;

    /** A topological path up-tree */
    using path_t = std::vector<node_t>;

    void insert_node_and_edge(
        const mrpt::graphs::TNodeID parent_id,
        const mrpt::graphs::TNodeID new_child_id,
        const NODE_TYPE_DATA&       new_child_node_data,
        const EDGE_TYPE&            new_edge_data)
    {
        // edge:
        typename base_t::TListEdges& edges_of_parent =
            base_t::edges_to_children[parent_id];
        edges_of_parent.push_back(typename base_t::TEdgeInfo(
            new_child_id, false /*direction_child_to_parent*/, new_edge_data));
        // node:
        nodes_[new_child_id] = node_t(
            new_child_id, parent_id, &edges_of_parent.back().data,
            new_child_node_data);
    }

    /** Insert a node without edges (should be used only for a tree root node)
     */
    void insert_node(
        const mrpt::graphs::TNodeID node_id, const NODE_TYPE_DATA& node_data)
    {
        nodes_[node_id] = node_t(node_id, INVALID_NODEID, nullptr, node_data);
    }

    mrpt::graphs::TNodeID next_free_node_ID() const { return nodes_.size(); }

    const node_map_t& nodes() const { return nodes_; }

    /** Builds the path (sequence of nodes, with info about next edge) up-tree
     * from a `target_node` towards the root
     * Nodes are ordered in the direction ROOT -> start_node
     */
    path_t backtrack_path(const mrpt::graphs::TNodeID target_node) const
    {
        path_t out_path;
        auto   it_src = nodes_.find(target_node);
        if (it_src == nodes_.end())
            throw std::runtime_error(
                "backtrackPath: target_node not found in tree!");
        const node_t* node = &it_src->second;
        for (;;)
        {
            out_path.push_front(*node);

            auto next_node_id = node->parentID_;
            if (!next_node_id.has_value())
            {
                // root reached: finished
                break;
            }
            else
            {
                auto it_next = nodes_.find(next_node_id.value());
                if (it_next == nodes_.end())
                    throw std::runtime_error(
                        "backtrackPath: Node ID not found during tree "
                        "traversal!");
                node = &it_next->second;
            }
        }
        return out_path;
    }

   private:
    /** Info per node */
    node_map_t nodes_;

};  // end TMoveTree

/** Pose metric for SE(2) limited to a given PTG manifold. NOTE: This 'metric'
 * is NOT symmetric for all PTGs: d(a,b)!=d(b,a) */
template <>
struct PoseDistanceMetric<SE2_KinState>
{
    // Note: ptg is not const since we'll need to update its dynamic state
    PoseDistanceMetric(ptg_t& ptg) : m_ptg(ptg) {}

    bool cannotBeNearerThan(
        const SE2_KinState& a, const mrpt::math::TPose2D& b,
        const distance_t d) const
    {
        if (std::abs(a.pose.x - b.x) > d) return true;
        if (std::abs(a.pose.y - b.y) > d) return true;
        if (std::abs(mrpt::math::angDistance(a.pose.phi, b.phi)) > d)
            return true;
        return false;
    }
    std::optional<std::tuple<distance_t, trajectory_index_t>> distance(
        const SE2_KinState& src, const mrpt::math::TPose2D& dst) const
    {
        double             d;
        trajectory_index_t k;
        const auto         relPose     = dst - src.pose;
        auto               localSrcVel = src.vel;
        localSrcVel.rotate(-src.pose.phi);

        ptg_t::TNavDynamicState dynState;
        dynState.relTarget      = relPose;
        dynState.targetRelSpeed = 1.0;  // TODO! (?)
        dynState.curVelLocal    = localSrcVel;

        m_ptg.updateNavDynamicState(dynState);

        bool tp_point_is_exact =
            m_ptg.inverseMap_WS2TP(relPose.x, relPose.y, k, d);
        if (tp_point_is_exact)
        {
            // de-normalize distance
            return {{d * m_ptg.getRefDistance(), k}};
        }
        else
        {
            // not in range: we can't evaluate this distance!
            return {};
        }
    }

   private:
    ptg_t& m_ptg;
};

/** tree data structure for planning in SE2 within TP-Space manifolds */
using MotionPrimitivesTreeSE2 =
    MotionPrimitivesTree<SE2_KinState, MoveEdgeSE2_TPS>;

/** @} */
}  // namespace selfdriving
