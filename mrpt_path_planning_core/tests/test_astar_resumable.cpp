/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Resumable TPS_Astar search (start(), run(), reroot(), set_obstacles()):
 *  - Searching in many short slices gives the same result as plan().
 *  - Before the search ends, run() returns the best partial path so far.
 *  - After reroot() to a node of the best path, the tree hangs from it,
 *    costs are relative to it, and the search still reaches the goal.
 *  - Obstacles given with set_obstacles() are avoided by later expansions.
 *  - first_colliding_edge() finds the edges of a path hit by new obstacles.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/path_collision_check.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <thread>
#include <vector>

namespace
{
const char* kPtgCfg = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0

PTG_COUNT = 2

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 4.0
PTG0_num_paths   = 41
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0

PTG1_Type        = mpp::ptg::DiffDrive_C
PTG1_resolution  = 0.10
PTG1_refDistance = 4.0
PTG1_num_paths   = 41
PTG1_v_max_mps   = 1.0
PTG1_w_max_dps   = 60.0
PTG1_K           = -1.0

RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

const mpp::TrajectoriesAndRobotShape& ptgs()
{
    static const mpp::TrajectoriesAndRobotShape trs = []()
    {
        mrpt::config::CConfigFileMemory cfg(kPtgCfg);
        mpp::TrajectoriesAndRobotShape  r;
        r.initFromConfigFile(cfg, "SelfDriving");
        return r;
    }();
    return trs;
}

// A wall at x=wallX, for y in [-halfLength, halfLength]:
mpp::ObstacleSource::Ptr wall(double wallX, double halfLength)
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (double y = -halfLength; y <= halfLength; y += 0.05)
    {
        pts->insertPoint(
            static_cast<float>(wallX), static_cast<float>(y), 0.0f);
    }
    return mpp::ObstacleSource::FromStaticPointcloud(pts);
}

mpp::PlannerInput makeInput(
    const std::vector<mpp::ObstacleSource::Ptr>& obstacles)
{
    mpp::PlannerInput in;
    in.ptgs            = ptgs();
    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPose2D{6.0, 0.0, 0.0};
    in.worldBboxMin    = {-3, -6, -M_PI};
    in.worldBboxMax    = {9, 6, M_PI};
    in.obstacles       = obstacles;
    return in;
}

void configure(mpp::TPS_Astar& planner)
{
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.profiler_().enable(false);
    planner.params_.maximumComputationTime = 60.0;
}

// Poses along the path to `nodeId`, every PTG step:
std::vector<mrpt::math::TPose2D> pathPoses(
    const mpp::PlannerOutput& po, mpp::TNodeID nodeId)
{
    std::vector<mrpt::math::TPose2D> out;
    const auto [nodes, edges] = po.motionTree.backtrack_path(nodeId);
    for (const auto* e : edges)
    {
        if (e == nullptr) { continue; }
        const auto& ptg = po.originalInput.ptgs.ptgs.at(e->ptgIndex);
        for (uint32_t step = 0; step <= e->ptgStepIndex; step++)
        {
            out.push_back(
                e->stateFrom.pose + ptg->getPathPose(e->ptgPathIndex, step));
        }
    }
    return out;
}

bool passesThrough(
    const std::vector<mrpt::math::TPose2D>& poses, double wallX,
    double halfLength)
{
    for (size_t i = 1; i < poses.size(); i++)
    {
        const auto& a = poses[i - 1];
        const auto& b = poses[i];
        if ((a.x - wallX) * (b.x - wallX) <= 0 && std::abs(a.y) < halfLength &&
            std::abs(b.y) < halfLength)
        {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(AstarResumable, SlicedSearchMatchesPlan)
{
    mpp::TPS_Astar oneShot;
    configure(oneShot);
    const auto ref = oneShot.plan(makeInput({wall(3.0, 2.0)}));
    ASSERT_TRUE(ref.success);

    mpp::TPS_Astar sliced;
    configure(sliced);
    sliced.start(makeInput({wall(3.0, 2.0)}));
    mpp::PlannerOutput po;
    size_t             slices = 0;
    while (sliced.search_can_continue())
    {
        // A zero time budget expands one node per call:
        po = sliced.run(0.0);
        slices++;
    }
    EXPECT_GT(slices, 10U);
    EXPECT_TRUE(po.success);
    EXPECT_TRUE(sliced.search_reached_goal());
    EXPECT_EQ(po.numExpandedNodes, ref.numExpandedNodes);
    EXPECT_DOUBLE_EQ(po.pathCost, ref.pathCost);
}

TEST(AstarResumable, AnytimeBestPartialPath)
{
    mpp::TPS_Astar planner;
    configure(planner);
    const auto in = makeInput({wall(3.0, 2.0)});
    planner.start(in);

    mpp::PlannerOutput po;
    for (int i = 0; i < 5; i++) { po = planner.run(0.0); }
    ASSERT_FALSE(po.success);
    ASSERT_TRUE(po.bestNodeId.has_value());
    EXPECT_TRUE(planner.search_can_continue());

    // The best node is closer to the goal than the start:
    const auto& best = po.motionTree.nodes().at(*po.bestNodeId);
    EXPECT_LT(
        (best.pose.translation() - in.stateGoal.state.pose().translation())
            .norm(),
        (in.stateStart.pose.translation() -
         in.stateGoal.state.pose().translation())
            .norm());

    // ...and the partial path to it is a chain of edges from the start:
    const auto [nodes, edges] = po.motionTree.backtrack_path(*po.bestNodeId);
    ASSERT_GE(nodes.size(), 2U);
    EXPECT_EQ(nodes.front().nodeID_, po.motionTree.root);
    EXPECT_EQ(nodes.back().nodeID_, *po.bestNodeId);
}

TEST(AstarResumable, RerootKeepsSubtreeAndReachesGoal)
{
    mpp::TPS_Astar planner;
    configure(planner);
    planner.start(makeInput({wall(3.0, 2.0)}));

    mpp::PlannerOutput po;
    for (int i = 0; i < 20 && planner.search_can_continue(); i++)
    {
        po = planner.run(0.0);
    }
    ASSERT_TRUE(po.bestNodeId.has_value());
    const auto [nodes, edges] = po.motionTree.backtrack_path(*po.bestNodeId);
    ASSERT_GE(nodes.size(), 2U);

    // The first node after the start along the best path:
    const auto newRoot     = std::next(nodes.begin())->nodeID_;
    const auto newRootPose = std::next(nodes.begin())->pose;

    EXPECT_FALSE(planner.reroot(999999));
    ASSERT_TRUE(planner.reroot(newRoot));

    po = planner.run(0.0);
    EXPECT_EQ(po.motionTree.root, newRoot);
    EXPECT_EQ(po.motionTree.nodes().at(newRoot).cost_, 0);
    EXPECT_FALSE(po.motionTree.nodes().at(newRoot).parentID_.has_value());
    EXPECT_DOUBLE_EQ(po.originalInput.stateStart.pose.x, newRootPose.x);
    EXPECT_DOUBLE_EQ(po.originalInput.stateStart.pose.y, newRootPose.y);
    for (const auto& [id, node] : po.motionTree.nodes())
    {
        if (id == newRoot) { continue; }
        ASSERT_TRUE(node.parentID_.has_value());
        EXPECT_NE(po.motionTree.nodes().count(*node.parentID_), 0U);
        EXPECT_GT(node.cost_, 0);
    }

    while (planner.search_can_continue()) { po = planner.run(1.0); }
    ASSERT_TRUE(po.success);
    const auto [solNodes, solEdges] =
        po.motionTree.backtrack_path(*po.goalNodeId);
    EXPECT_EQ(solNodes.front().nodeID_, newRoot);
    EXPECT_FALSE(passesThrough(pathPoses(po, *po.goalNodeId), 3.0, 2.0));
}

TEST(AstarResumable, RerootAfterReachingGoalKeepsSolution)
{
    mpp::TPS_Astar planner;
    configure(planner);
    planner.start(makeInput({}));
    auto po = planner.run(10.0);
    ASSERT_TRUE(po.success);
    const auto [nodes, edges] = po.motionTree.backtrack_path(*po.goalNodeId);
    ASSERT_GE(nodes.size(), 2U);
    const auto newRoot = std::next(nodes.begin())->nodeID_;

    ASSERT_TRUE(planner.reroot(newRoot));
    EXPECT_TRUE(planner.search_reached_goal());
    po = planner.run(1.0);
    EXPECT_TRUE(po.success);
    const auto [solNodes, solEdges] =
        po.motionTree.backtrack_path(*po.goalNodeId);
    EXPECT_EQ(solNodes.front().nodeID_, newRoot);
}

TEST(AstarResumable, NewObstaclesAreAvoided)
{
    mpp::TPS_Astar planner;
    configure(planner);
    planner.start(makeInput({}));

    // A wall appears before the search expands anything:
    planner.set_obstacles({wall(3.0, 2.0)});
    mpp::PlannerOutput po;
    while (planner.search_can_continue()) { po = planner.run(1.0); }
    ASSERT_TRUE(po.success);
    EXPECT_FALSE(passesThrough(pathPoses(po, *po.goalNodeId), 3.0, 2.0));
}

TEST(AstarResumable, FirstCollidingEdge)
{
    mpp::TPS_Astar planner;
    configure(planner);
    const auto po = planner.plan(makeInput({}));
    ASSERT_TRUE(po.success);

    const auto [nodes, edgeList] = po.motionTree.backtrack_path(*po.goalNodeId);
    std::vector<const mpp::MoveEdgeSE2_TPS*> edges(
        edgeList.begin(), edgeList.end());
    ASSERT_GE(edges.size(), 1U);

    const auto& trs = po.originalInput.ptgs;
    EXPECT_FALSE(mpp::first_colliding_edge(edges, trs, {}).has_value());

    // Obstacles far from the path:
    auto far = mrpt::maps::CSimplePointsMap::Create();
    far->insertPoint(3.0f, 5.0f, 0.0f);
    EXPECT_FALSE(mpp::first_colliding_edge(edges, trs, {far}).has_value());

    // A wall across the path, at x=3: the first colliding edge is the one
    // reaching it.
    auto wallPts = mrpt::maps::CSimplePointsMap::Create();
    for (double y = -2.0; y <= 2.0; y += 0.05)
    {
        wallPts->insertPoint(3.0f, static_cast<float>(y), 0.0f);
    }
    const auto hit = mpp::first_colliding_edge(edges, trs, {wallPts});
    ASSERT_TRUE(hit.has_value());
    size_t expected = 0;
    while (expected < edges.size() &&
           edges[expected]->stateTo.pose.x < 3.0 - 0.5)
    {
        expected++;
    }
    EXPECT_EQ(*hit, expected);
}

// Re-rooting repeatedly, along the best path, while the search goes on (as a
// navigator does for a moving robot): the search must stay consistent.
TEST(AstarResumable, RepeatedRerootWhileSearching)
{
    for (const double eps : {1.0, 2.0})
    {
        mpp::TPS_Astar planner;
        configure(planner);
        planner.params_.heuristic_epsilon         = eps;
        planner.params_.use_reeds_shepp_heuristic = true;
        planner.params_.use_reeds_shepp_expansion = true;
        planner.start(makeInput({wall(3.0, 2.0), wall(1.5, 0.6)}));

        mpp::PlannerOutput po;
        for (int i = 0; i < 400 && planner.search_can_continue(); i++)
        {
            ASSERT_NO_THROW(po = planner.run(0.0))
                << "eps=" << eps << " i=" << i;
            if (i % 7 == 6 && po.bestNodeId)
            {
                const auto [nodes, edges] =
                    po.motionTree.backtrack_path(*po.bestNodeId);
                if (nodes.size() >= 3)
                {
                    ASSERT_NO_THROW(
                        planner.reroot(std::next(nodes.begin())->nodeID_));
                }
            }
        }
        while (planner.search_can_continue())
        {
            ASSERT_NO_THROW(po = planner.run(1.0)) << "eps=" << eps;
        }
        EXPECT_TRUE(po.success) << "eps=" << eps;
    }
}

TEST(AstarResumable, RunSliceAndPathToNodeMatchRun)
{
    mpp::TPS_Astar a;
    mpp::TPS_Astar b;
    configure(a);
    configure(b);
    a.start(makeInput({wall(3.0, 2.0)}));
    b.start(makeInput({wall(3.0, 2.0)}));

    for (int i = 0; i < 20; i++)
    {
        const auto po = a.run(0.0);
        const auto sl = b.run_slice(0.0);
        EXPECT_EQ(po.success, sl.success);
        EXPECT_EQ(po.bestNodeId, sl.bestNodeId);
        EXPECT_EQ(po.numExpandedNodes, sl.numExpandedNodes);
        EXPECT_DOUBLE_EQ(po.pathCost, sl.pathCost);
    }
    const auto sl = b.run_slice(0.0);
    ASSERT_TRUE(sl.bestNodeId.has_value());
    const auto [nodes, edges] = b.path_to_node(*sl.bestNodeId);
    ASSERT_FALSE(edges.empty());
    for (const auto* e : edges)
    {
        ASSERT_NE(e, nullptr);
        EXPECT_GE(e->interpolatedPath.size(), 2U);
    }
}

TEST(AstarResumable, CopiedPlannerIsIndependent)
{
    std::unique_ptr<mpp::TPS_Astar> orig = std::make_unique<mpp::TPS_Astar>();
    configure(*orig);
    orig->params_.heuristic_epsilon = 1.25;
    orig->start(makeInput({wall(3.0, 2.0)}));
    for (int i = 0; i < 5; i++) { orig->run(0.0); }

    mpp::TPS_Astar copy(*orig);
    EXPECT_DOUBLE_EQ(copy.params_.heuristic_epsilon, 1.25);
    EXPECT_FALSE(copy.search_can_continue());  // no search state is copied
    orig.reset();  // the copy must not depend on the source

    const auto po = copy.plan(makeInput({wall(3.0, 2.0)}));
    EXPECT_TRUE(po.success);
}

TEST(AstarResumable, ParallelPlannersOnClonedPtgs)
{
    constexpr int            kThreads = 4;
    std::vector<bool>        ok(kThreads, false);
    std::vector<double>      costs(kThreads, 0.0);
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; t++)
    {
        threads.emplace_back(
            [&, t]()
            {
                mpp::TPS_Astar planner;
                configure(planner);
                auto in = makeInput({wall(3.0, 2.0)});
                in.ptgs = ptgs().clone();  // nothing shared but the obstacles
                const auto po = planner.plan(in);
                ok[t]         = po.success;
                costs[t]      = po.pathCost;
            });
    }
    for (auto& th : threads) { th.join(); }
    for (int t = 0; t < kThreads; t++)
    {
        EXPECT_TRUE(ok[t]) << t;
        EXPECT_DOUBLE_EQ(costs[t], costs[0]) << t;
    }
}

namespace
{
const char* kHolonomicCfg = R"cfg(
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
}  // namespace

TEST(AstarResumable, HolonomicStartRunRerootSetObstacles)
{
    mpp::PlannerInput in;
    {
        mrpt::config::CConfigFileMemory cfg(kHolonomicCfg);
        in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    }
    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPose2D{6.0, 0.0, 0.0};
    in.worldBboxMin    = {-3, -6, -M_PI};
    in.worldBboxMax    = {9, 6, M_PI};
    in.obstacles       = {wall(3.0, 1.0)};

    mpp::TPS_Astar planner;
    configure(planner);
    planner.start(in);

    mpp::PlannerOutput po;
    for (int i = 0; i < 10; i++) { po = planner.run(0.0); }
    ASSERT_TRUE(po.bestNodeId.has_value());

    // Reroot one node ahead on the best path:
    const auto [nodes, edges] = po.motionTree.backtrack_path(*po.bestNodeId);
    ASSERT_GE(nodes.size(), 3U);
    ASSERT_TRUE(planner.reroot(std::next(nodes.begin())->nodeID_));

    // New obstacles for the rest of the search:
    planner.set_obstacles({wall(3.0, 1.0), wall(4.5, 1.0)});
    while (planner.search_can_continue()) { po = planner.run(1.0); }
    EXPECT_TRUE(po.success);
}
