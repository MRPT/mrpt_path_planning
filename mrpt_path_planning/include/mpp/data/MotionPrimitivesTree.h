/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/SE2_KinState.h>
#include <mpp/data/basic_types.h>
#include <mpp/data/ptg_t.h>
#include <mrpt/containers/traits_map.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/graphs/CDirectedTree.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/poses/CPose2D.h>

#include <cstdint>
#include <deque>
#include <list>
#include <optional>
#include <set>
#include <tuple>

namespace mpp
{
using mrpt::graphs::TNodeID;

/** Generic base for metrics */
template <class node_t>
struct PoseDistanceMetric_TPS;

template <class node_t>
struct PoseDistanceMetric_Lie;

struct TPS_point
{
    TPS_point()  = default;
    ~TPS_point() = default;

    TPS_point(
        trajectory_index_t _k, ptg_step_t _step, normalized_speed_t _speed)
        : k(_k), step(_step), speed(_speed)
    {
    }

    trajectory_index_t k;
    ptg_step_t         step;
    normalized_speed_t speed;
};
inline bool operator<(const TPS_point& a, const TPS_point& b)
{
    return (a.k < b.k) || (a.k == b.k && a.step < b.step) ||
           (a.k == b.k && a.step == b.step && a.speed < b.speed);
}

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
template <class NODE_TYPE_DATA, class EDGE_TYPE>
class MotionPrimitivesTree : public mrpt::graphs::CDirectedTree<EDGE_TYPE>
{
   public:
    struct node_t : public NODE_TYPE_DATA
    {
        /** Duplicated ID (it's also in the map::iterator->first), but put here
         * to make it available in path_t */
        TNodeID nodeID_ = mrpt::graphs::INVALID_NODEID;

        /** Does not have value for the root, a valid ID otherwise */
        std::optional<TNodeID> parentID_;

        /** cost of reaching this node from the root (=0 for the root) */
        cost_t cost_;

        node_t() = default;
        node_t(
            TNodeID nodeID, const std::optional<TNodeID>& parentID,
            const NODE_TYPE_DATA& data, cost_t cost)
            : NODE_TYPE_DATA(data),
              nodeID_(nodeID),
              parentID_(parentID),
              cost_(cost)
        {
        }
    };

    using base_t          = mrpt::graphs::CDirectedTree<EDGE_TYPE>;
    using edge_t          = EDGE_TYPE;
    using edge_sequence_t = std::list<edge_t*>;

    /**  Use deque to reduce memory reallocs. */
    struct map_traits_map_as_deque
    {
        template <class KEY, class VALUE>
        using map = mrpt::containers::map_as_vector<
            KEY, VALUE, typename std::deque<std::pair<KEY, VALUE>>>;
    };

    /** Map: TNode_ID => Node info */
    using node_map_t = std::map<TNodeID, node_t>;
    // typename map_traits_map_as_deque::template map<TNodeID, node_t>;

    /** A topological path up-tree.
     *
     *  \note We use std::list since we need a container with `push_front()`.
     */
    using path_t = std::list<node_t>;

    void insert_node_and_edge(
        const TNodeID parentId, const TNodeID newChildId,
        const NODE_TYPE_DATA& newChildNodeData, const EDGE_TYPE& newEdgeData)
    {
        // edge:
        auto&      parentEdges      = base_t::edges_to_children[parentId];
        const bool dirChildToParent = false;

        parentEdges.emplace_back(newChildId, dirChildToParent, newEdgeData);

        const cost_t newCost = nodes_.at(parentId).cost_ + newEdgeData.cost;

        // node:
        nodes_[newChildId] =
            node_t(newChildId, parentId, newChildNodeData, newCost);
    }

    void update_node_and_edge(
        const TNodeID parentId, const TNodeID childId,
        const EDGE_TYPE& newEdgeData)
    {
        // edge:
        auto& parentEdges = base_t::edges_to_children[parentId];
        bool  updateDone  = false;
        // const bool dirChildToParent = false;
        for (auto& edge : parentEdges)
        {
            if (edge.id == childId)
            {
                edge.data  = newEdgeData;
                updateDone = true;
                break;
            }
        }
        if (!updateDone)
        {
            THROW_EXCEPTION_FMT(
                "[update_node_and_edge] Error: Could not find edge from "
                "parent #%s -> node #%s",
                std::to_string(parentId).c_str(),
                std::to_string(childId).c_str());
        }

        const cost_t newCost = nodes_.at(parentId).cost_ + newEdgeData.cost;

        // node:
        auto& node     = nodes_.at(childId);
        node.parentID_ = parentId;
        node.cost_     = newCost;
    }

    void rewire_node_parent(
        const TNodeID nodeId, const EDGE_TYPE& newEdgeFromParent)
    {
        auto& node = nodes_.at(nodeId);

        // Remove old edge:
        const auto oldParentId    = *node.parentID_;
        auto&      oldParentEdges = base_t::edges_to_children[oldParentId];
        bool       deletionDone   = false;
        for (auto it = oldParentEdges.begin(); it != oldParentEdges.end(); ++it)
        {
            if (it->id == nodeId)
            {
                oldParentEdges.erase(it);
                deletionDone = true;
                break;
            }
        }
        if (!deletionDone)
        {
            THROW_EXCEPTION_FMT(
                "[rewire_node_parent] Error: Could not find edge from former "
                "parent #%s -> node #%s",
                std::to_string(oldParentId).c_str(),
                std::to_string(nodeId).c_str());
        }

        // add edge:
        const TNodeID                parentId = newEdgeFromParent.parentId;
        typename base_t::TListEdges& parentEdges =
            base_t::edges_to_children[parentId];
        const bool dirChildToParent = false;
        parentEdges.emplace_back(nodeId, dirChildToParent, newEdgeFromParent);

        const cost_t newCost =
            nodes_.at(parentId).cost_ + newEdgeFromParent.cost;

        // update existing node info:
        ASSERT_LE_(newCost, node.cost_);

        node.parentID_ = parentId;
        node.cost_     = newCost;
    }

    const EDGE_TYPE& edge_to_parent(const TNodeID nodeId) const
    {
        auto&       node        = nodes_.at(nodeId);
        const auto& parentEdges = base_t::edges_to_children.at(*node.parentID_);
        for (const auto& edge : parentEdges)
        {
            if (edge.id == nodeId) return edge.data;
        }
        THROW_EXCEPTION_FMT(
            "Could not find edge to parent for node #%s",
            std::to_string(nodeId).c_str());
    }

    /** Insert a node without edges (should be used only for a tree root node)
     */
    void insert_root_node(
        const TNodeID node_id, const NODE_TYPE_DATA& node_data)
    {
        ASSERTMSG_(
            nodes_.empty(), "insert_root_node() called on a non-empty tree");
        cost_t zeroCost = 0;
        nodes_[node_id] = node_t(node_id, {}, node_data, zeroCost);
    }

    TNodeID next_free_node_ID() const { return nodes_.size(); }

    /** read-only access to nodes.
     * \sa  insert_node_and_edge, insert_node
     */
    const node_map_t& nodes() const { return nodes_; }

    /** Write-access to node data (use with caution) */
    NODE_TYPE_DATA& node_state(const TNodeID nodeId)
    {
        return nodes_.at(nodeId);
    }

    /** Builds the path (sequence of nodes, with info about next edge) up-tree
     * from a `target_node` towards the root
     * - path_t nodes are ordered in the direction ROOT -> start_node
     * - ed
     */
    std::tuple<path_t, edge_sequence_t> backtrack_path(
        const TNodeID target_node) const
    {
        path_t          outPath;
        edge_sequence_t edgeList;

        auto it_src = nodes_.find(target_node);
        if (it_src == nodes_.end())
            throw std::runtime_error(
                "backtrackPath: target_node not found in tree!");
        const node_t* node = &it_src->second;
        for (;;)
        {
            outPath.push_front(*node);

            auto next_node_id = node->parentID_;
            if (!next_node_id.has_value())
            {
                // root reached: finished
                break;
            }
            else
            {
                const EDGE_TYPE& edge = edge_to_parent(node->nodeID_);
                edgeList.push_front(const_cast<EDGE_TYPE*>(&edge));

                auto it_next = nodes_.find(next_node_id.value());
                if (it_next == nodes_.end())
                    throw std::runtime_error(
                        "backtrackPath: Node ID not found during tree "
                        "traversal!");
                node = &it_next->second;
            }
        }
        return {outPath, edgeList};
    }

   private:
    /** Info per node */
    node_map_t nodes_;

};  // end TMoveTree

/** Pose metric for SE(2) limited to a given PTG manifold. NOTE: This 'metric'
 * is NOT symmetric for all PTGs: d(a,b)!=d(b,a) */
template <>
struct PoseDistanceMetric_TPS<SE2_KinState>
{
    // Note: ptg is not const since we'll need to update its dynamic state
    PoseDistanceMetric_TPS(ptg_t& ptg, const double headingTolerance)
        : ptg_(ptg), headingTolerance_(headingTolerance)
    {
    }

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
        const SE2_KinState& src, const mrpt::math::TPose2D& dst,
        bool ignoreDstHeading) const
    {
        normalized_distance_t normDist;
        trajectory_index_t    k;
        const auto            relPose     = dst - src.pose;
        auto                  localSrcVel = src.vel;
        localSrcVel.rotate(-src.pose.phi);

        ptg_t::TNavDynamicState dynState;
        dynState.relTarget      = relPose;
        dynState.targetRelSpeed = 1.0;  // TODO! (?)
        dynState.curVelLocal    = localSrcVel;

        ptg_.updateNavDynamicState(dynState);

        bool tp_point_is_exact =
            ptg_.inverseMap_WS2TP(relPose.x, relPose.y, k, normDist);

        distance_t d = normDist * ptg_.getRefDistance();

        if (tp_point_is_exact)
        {
            uint32_t ptg_step;
            ptg_.getPathStepForDist(k, d, ptg_step);
            const auto   reconsRelPose = ptg_.getPathPose(k, ptg_step);
            const double headingError =
                ignoreDstHeading ? .0
                                 : std::abs(mrpt::math::angDistance(
                                       reconsRelPose.phi, relPose.phi));

            if (headingError > headingTolerance_) tp_point_is_exact = false;
        }

        if (tp_point_is_exact)
        {
            // de-normalize distance
            if (d == 0 &&
                (relPose.x != 0 || relPose.y != 0 || relPose.phi != 0))
            {
                // Due to the discrete nature of PTG paths, in rare cases
                // we have d=0 despite the target is actually not exactly, but
                // very close to the origin:
                d = relPose.norm() +
                    std::abs(relPose.phi) * ptg_.getRefDistance();
            }
            return {{d, k}};
        }
        else
        {
            // not in range: we can't evaluate this distance!
            return {};
        }
    }

   private:
    ptg_t&       ptg_;
    const double headingTolerance_;
};

/** Pose metric for SE(2) on the actual Lie group, i.e. NOT limited to a given
 * PTG manifold, and ignoring velocities. */
template <>
struct PoseDistanceMetric_Lie<SE2_KinState>
{
    // Note: ptg is not const since we'll need to update its dynamic state
    PoseDistanceMetric_Lie(const double phiWeight) : phiWeight_(phiWeight) {}

    bool cannotBeNearerThan(
        const mrpt::math::TPose2D& a, const mrpt::math::TPose2D& b,
        const distance_t d) const
    {
        if (std::abs(a.x - b.x) > d) return true;
        if (std::abs(a.y - b.y) > d) return true;
        if (phiWeight_ * std::abs(mrpt::math::angDistance(a.phi, b.phi)) > d)
            return true;
        return false;
    }

    distance_t distance(
        const mrpt::math::TPose2D& src, const mrpt::math::TPose2D& dst) const
    {
        const auto relPose = dst - src;
        return relPose.norm() + phiWeight_ * std::abs(relPose.phi);
    }

   private:
    double phiWeight_ = 0.1;
};

/** tree data structure for planning in SE2 within TP-Space manifolds */
using MotionPrimitivesTreeSE2 =
    MotionPrimitivesTree<SE2_KinState, MoveEdgeSE2_TPS>;

/** @} */
}  // namespace mpp
