/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for refine_trajectory().
 *
 * Verified behaviors:
 *  - Single edge: after refinement the reconstructed PTG pose matches the
 *    actual node delta within tolerance.
 *  - Multi-edge: every edge is independently corrected.
 *  - Zero-delta node pair: skipped silently, no crash.
 *  - Both overloads (path_t/edge_sequence_t and vector) produce the same result.
 */

#include <gtest/gtest.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/config/CConfigFileMemory.h>

static const char* kPtgCfg = R"cfg(
[SelfDriving]
min_obstacles_height  = 0.0
max_obstacles_height  = 2.0

PTG_COUNT = 1

PTG0_Type        = mpp::ptg::HolonomicBlend
PTG0_refDistance = 5.0
PTG0_num_paths   = 61
PTG0_T_ramp_max  = 1.0
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_expr_V      = V_MAX * trimmable_speed
PTG0_expr_W      = W_MAX * trimmable_speed * min(1.0, 0.1+abs(dir)/(10*3.14159265/180))
PTG0_expr_T_ramp = T_ramp_max

RobotModel_circular_shape_radius = 0.15
)cfg";

using NodeT = mpp::MotionPrimitivesTreeSE2::node_t;
using EdgeT = mpp::MotionPrimitivesTreeSE2::edge_t;  // = MoveEdgeSE2_TPS

static mpp::TrajectoriesAndRobotShape buildTRS()
{
    mrpt::config::CConfigFileMemory cfg(kPtgCfg);
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    return trs;
}

static NodeT makeNode(double x, double y, double phi = 0.0)
{
    mpp::SE2_KinState s;
    s.pose = {x, y, phi};
    return NodeT(0, {}, s, 0.0);
}

static EdgeT makeEdge(
    int8_t ptgIdx = 0, int16_t pathIdx = 0, double dist = 999.0,
    mrpt::math::TPose2D relGoal = {1.0, 0.0, 0.0})
{
    EdgeT e;
    e.ptgIndex             = ptgIdx;
    e.ptgPathIndex         = pathIdx;
    e.ptgDist              = dist;
    e.stateFrom.pose       = {0, 0, 0};
    e.ptgFinalRelativeGoal = relGoal;  // non-zero: required by HolonomicBlend
    // refine_trajectory calls edge_interpolated_path(numSegments=nullopt),
    // which requires at least 2 entries already in the path.
    e.interpolatedPath[0.0] = {0, 0, 0};
    e.interpolatedPath[1.0] = relGoal;
    return e;
}

// Returns true if the PTG-reconstructed pose for this edge is within tol of target.
static bool poseClose(
    const mpp::TrajectoriesAndRobotShape& trs, const EdgeT& edge,
    const mrpt::math::TPose2D& target, double tol = 0.10)
{
    auto& ptg = trs.ptgs.at(edge.ptgIndex);
    ptg->updateNavDynamicState(edge.getPTGDynState());
    uint32_t step = 0;
    ptg->getPathStepForDist(edge.ptgPathIndex, edge.ptgDist, step);
    return (ptg->getPathPose(edge.ptgPathIndex, step) - target).norm() < tol;
}

// ---------------------------------------------------------------------------

TEST(RefineTrajectory, SingleEdge)
{
    auto trs = buildTRS();

    // Diagonal delta: (1.0, 0.5) should map to a non-trivial PTG trajectory.
    const mrpt::math::TPose2D delta{1.0, 0.5, 0.0};

    std::vector<NodeT> path  = {makeNode(0, 0), makeNode(delta.x, delta.y)};
    std::vector<EdgeT> edges = {makeEdge(0, 0, 999.0, {delta.x, delta.y, 0})};

    mpp::refine_trajectory(path, edges, trs, /*tol=*/0.5);

    EXPECT_TRUE(poseClose(trs, edges[0], delta))
        << "After refinement, reconstructed pose must match node delta";
}

TEST(RefineTrajectory, MultiEdge)
{
    auto trs = buildTRS();

    const mrpt::math::TPose2D delta1{1.0, 0.0, 0.0};
    const mrpt::math::TPose2D delta2{0.0, 1.0, 0.0};

    std::vector<NodeT> path = {
        makeNode(0, 0), makeNode(delta1.x, delta1.y),
        makeNode(delta1.x + delta2.x, delta1.y + delta2.y)};
    std::vector<EdgeT> edges = {
        makeEdge(0, 0, 999.0, {delta1.x, delta1.y, 0}),
        makeEdge(0, 0, 999.0, {delta2.x, delta2.y, 0})};

    mpp::refine_trajectory(path, edges, trs, 0.5);

    EXPECT_TRUE(poseClose(trs, edges[0], delta1))
        << "First edge must be corrected";
    EXPECT_TRUE(poseClose(trs, edges[1], delta2))
        << "Second edge must be corrected";
}

TEST(RefineTrajectory, ZeroDelta)
{
    auto trs = buildTRS();

    // Two nodes at the same position → delta = 0, edge is skipped.
    std::vector<NodeT> path  = {makeNode(0, 0), makeNode(0, 0)};
    std::vector<EdgeT> edges = {makeEdge(0, 0, 999.0, {1.0, 0.0, 0.0})};

    EXPECT_NO_THROW(mpp::refine_trajectory(path, edges, trs, 0.5))
        << "Zero-delta node pair must be skipped without throwing";

    // Edge should be unchanged (ptgPathIndex still 0, dist still 999).
    EXPECT_EQ(edges[0].ptgPathIndex, 0);
    EXPECT_NEAR(edges[0].ptgDist, 999.0, 1e-9);
}

TEST(RefineTrajectory, VectorOverloadMatchesPathOverload)
{
    auto trs = buildTRS();

    const mrpt::math::TPose2D delta{1.0, 0.3, 0.0};

    // Vector overload:
    std::vector<NodeT> path1  = {makeNode(0, 0), makeNode(delta.x, delta.y)};
    std::vector<EdgeT> edges1 = {makeEdge(0, 0, 999.0, {delta.x, delta.y, 0})};
    mpp::refine_trajectory(path1, edges1, trs, 0.5);

    // path_t / edge_sequence_t overload:
    mpp::MotionPrimitivesTreeSE2::path_t path2;
    for (const auto& n : path1) path2.push_back(n);

    EdgeT rawEdge = makeEdge(0, 0, 999.0, {delta.x, delta.y, 0});
    mpp::MotionPrimitivesTreeSE2::edge_sequence_t edges2;
    edges2.push_back(&rawEdge);
    mpp::refine_trajectory(path2, edges2, trs, 0.5);

    EXPECT_EQ(edges1[0].ptgPathIndex, rawEdge.ptgPathIndex)
        << "Both overloads must produce the same ptgPathIndex";
    EXPECT_NEAR(edges1[0].ptgDist, rawEdge.ptgDist, 1e-9)
        << "Both overloads must produce the same ptgDist";
}
