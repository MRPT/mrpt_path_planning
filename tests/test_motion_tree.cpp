/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for MotionPrimitivesTree operations.
 *
 * Verified behaviors:
 *  - insert_node_and_edge accumulates costs correctly.
 *  - backtrack_path returns nodes in root-to-target order.
 *  - rewire_node_parent re-parents a node and updates cost.
 *  - After rewire, backtrack_path reflects the new parent chain.
 *  - edge_to_parent returns the correct edge data.
 */

#include <gtest/gtest.h>
#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/SE2_KinState.h>

using Tree = mpp::MotionPrimitivesTreeSE2;

static mpp::SE2_KinState makeState(double x, double y, double phi = 0.0)
{
    mpp::SE2_KinState s;
    s.pose = {x, y, phi};
    return s;
}

static mpp::MoveEdgeSE2_TPS makeEdge(mpp::TNodeID parent, double cost)
{
    mpp::MoveEdgeSE2_TPS e;
    e.parentId = parent;
    e.cost     = cost;
    e.ptgIndex = 0;
    return e;
}

// ---------------------------------------------------------------------------

TEST(MotionTree, InsertAndBacktrack)
{
    Tree tree;

    // Build: root(0) → node(1) → node(2)
    tree.insert_root_node(0, makeState(0, 0));
    tree.insert_node_and_edge(0, 1, makeState(1, 0), makeEdge(0, 1.0));
    tree.insert_node_and_edge(1, 2, makeState(2, 0), makeEdge(1, 1.0));

    EXPECT_NEAR(tree.nodes().at(0).cost_, 0.0, 1e-9);
    EXPECT_NEAR(tree.nodes().at(1).cost_, 1.0, 1e-9);
    EXPECT_NEAR(tree.nodes().at(2).cost_, 2.0, 1e-9);

    auto [path, edges] = tree.backtrack_path(2);

    // Path must be root → 1 → 2 (three nodes, two edges)
    ASSERT_EQ(path.size(), 3u);
    auto it = path.begin();
    EXPECT_EQ((it++)->nodeID_, 0u);
    EXPECT_EQ((it++)->nodeID_, 1u);
    EXPECT_EQ((it++)->nodeID_, 2u);

    ASSERT_EQ(edges.size(), 2u);
}

TEST(MotionTree, RewireChangesPath)
{
    Tree tree;

    // Build: root(0) → node(1) → node(2)  (cost of node 2 = 2.0)
    tree.insert_root_node(0, makeState(0, 0));
    tree.insert_node_and_edge(0, 1, makeState(1, 0), makeEdge(0, 1.0));
    tree.insert_node_and_edge(1, 2, makeState(2, 0), makeEdge(1, 1.0));

    // Rewire node 2 directly under root with cost 1.5 < 2.0
    tree.rewire_node_parent(2, makeEdge(0, 1.5));

    EXPECT_NEAR(tree.nodes().at(2).cost_, 1.5, 1e-9);

    auto [path, edges] = tree.backtrack_path(2);

    // Path must now be root → 2 (two nodes, one edge — node 1 is bypassed)
    ASSERT_EQ(path.size(), 2u);
    auto it = path.begin();
    EXPECT_EQ((it++)->nodeID_, 0u);
    EXPECT_EQ((it++)->nodeID_, 2u);

    ASSERT_EQ(edges.size(), 1u);
}

TEST(MotionTree, EdgeToParent)
{
    Tree tree;

    tree.insert_root_node(0, makeState(0, 0));
    tree.insert_node_and_edge(0, 1, makeState(1, 0), makeEdge(0, 2.5));

    const auto& e = tree.edge_to_parent(1);
    EXPECT_NEAR(e.cost, 2.5, 1e-9);
    EXPECT_EQ(e.parentId, 0u);
}
