/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Misc. library checks:
 *  - getPathStepForDist() returns the first step reaching a distance.
 *  - Cost evaluators charge per unit of time, not per edge.
 *  - Static obstacle sources can be clipped from several threads.
 *  - TPS_Astar parameters round-trip through YAML.
 *  - The planner progress callback is called during a plan.
 *  - Following paths keeps their local-frame twist in the global frame.
 */

#include <gtest/gtest.h>
#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/trajectories.h>
#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <thread>

namespace
{
const char* kCfg = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0

PTG_COUNT = 1

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 4.0
PTG0_num_paths   = 41
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0

RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

mpp::TrajectoriesAndRobotShape buildTrs()
{
    mrpt::config::CConfigFileMemory cfg(kCfg);
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    return trs;
}
}  // namespace

TEST(Followups, PathStepForDistIsFirstStepReachingIt)
{
    auto       trs = buildTrs();
    const auto ptg = trs.ptgs.at(0);
    const auto k   = static_cast<uint16_t>(ptg->getPathCount() / 2);

    for (uint32_t step : {1U, 7U, 20U})
    {
        const double d  = ptg->getPathDist(k, step);
        const auto   st = ptg->getPathStepForDist(k, d);
        ASSERT_TRUE(st.has_value());
        EXPECT_EQ(*st, step);
    }
    // Slightly before a sample: that sample is the first one reaching it.
    const double d = ptg->getPathDist(k, 10) - 1e-6;
    EXPECT_EQ(*ptg->getPathStepForDist(k, d), 10U);

    // Out of reach:
    const auto last = ptg->getPathStepCount(k) - 1;
    EXPECT_FALSE(ptg->getPathStepForDist(k, ptg->getPathDist(k, last) + 1.0)
                     .has_value());
}

TEST(Followups, PlanToTrajectoryEdgeEndsAtItsFinalStep)
{
    auto       trs  = buildTrs();
    const auto ptg  = trs.ptgs.at(0);
    const auto k    = static_cast<uint16_t>(ptg->getPathCount() / 2);
    const auto step = 30U;

    mpp::MoveEdgeSE2_TPS edge;
    edge.ptgIndex       = 0;
    edge.ptgPathIndex   = static_cast<int16_t>(k);
    edge.ptgDist        = ptg->getPathDist(k, step);
    edge.stateFrom.pose = {0, 0, 0};
    edge.stateTo.pose   = ptg->getPathPose(k, step);

    mpp::MotionPrimitivesTreeSE2::edge_sequence_t seq;
    seq.push_back(&edge);
    const auto tr =
        mpp::plan_to_trajectory(seq, trs, ptg->getPathStepDuration());
    ASSERT_FALSE(tr.empty());
    EXPECT_NEAR(tr.rbegin()->first, step * ptg->getPathStepDuration(), 1e-9);
}

TEST(Followups, CostMapChargesPerUnitOfTime)
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->insertPoint(0.0f, 0.0f, 0.0f);
    mpp::CostEvaluatorCostMap::Parameters p;
    p.maxCost                    = 1.0;
    p.preferredClearanceDistance = 1.0;
    p.resolution                 = 0.05;
    const auto ev =
        mpp::CostEvaluatorCostMap::FromStaticPointObstacles(*pts, p);

    auto mkEdge = [](double T)
    {
        mpp::MoveEdgeSE2_TPS e;
        e.stateFrom.pose        = {0.3, 0, 0};
        e.interpolatedPath[0.0] = {0, 0, 0};
        e.interpolatedPath[T]   = {0.1, 0, 0};
        e.estimatedExecTime     = T;
        return e;
    };
    const double c1 = (*ev)(mkEdge(1.0));
    const double c5 = (*ev)(mkEdge(5.0));
    EXPECT_GT(c1, 0.0);
    EXPECT_NEAR(c5, 5.0 * c1, 1e-9);
}

TEST(Followups, StaticObstacleSourceConcurrentClipping)
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (int i = 0; i < 2000; i++)
    {
        pts->insertPoint(0.01f * static_cast<float>(i), 0.0f, 0.0f);
    }
    auto src = mpp::ObstacleSource::FromStaticPointcloud(pts);

    std::vector<std::thread> threads;
    for (int t = 0; t < 4; t++)
    {
        threads.emplace_back(
            [&]()
            {
                for (int i = 0; i < 50; i++)
                {
                    src->apply_clipping_box(mrpt::math::TBoundingBox(
                        {0.0, -1.0, -1.0}, {5.0, 1.0, 1.0}));
                    const auto o = src->obstacles();
                    EXPECT_GT(o->size(), 0U);
                    EXPECT_LE(o->size(), 2000U);
                }
            });
    }
    for (auto& th : threads) { th.join(); }

    src->apply_clipping_box(
        mrpt::math::TBoundingBox({0.0, -1.0, -1.0}, {5.0, 1.0, 1.0}));
    EXPECT_NEAR(static_cast<double>(src->obstacles()->size()), 501.0, 2.0);
    src->apply_clipping_box(std::nullopt);
    EXPECT_EQ(src->obstacles()->size(), 2000U);
}

TEST(Followups, GenericSensorObstacleSourceClipsRawCloud)
{
    // No observation: empty cloud, with and without clipping
    mpp::ObstacleSourceGenericSensor src;
    EXPECT_EQ(src.obstacles()->size(), 0U);
    src.apply_clipping_box(
        mrpt::math::TBoundingBox({0.0, -1.0, -1.0}, {5.0, 1.0, 1.0}));
    EXPECT_EQ(src.obstacles()->size(), 0U);
    EXPECT_EQ(src.get_stored_sensor_observation(), nullptr);
}

TEST(Followups, AstarParamsYamlRoundTrip)
{
    mpp::TPS_Astar a;
    a.params_.heuristic_epsilon      = 1.75;
    a.params_.grid_resolution_xy     = 0.31;
    a.params_.maximumComputationTime = 12.5;
    const auto y                     = a.params_as_yaml();

    mpp::TPS_Astar b;
    b.params_from_yaml(y);
    EXPECT_DOUBLE_EQ(b.params_.heuristic_epsilon, 1.75);
    EXPECT_DOUBLE_EQ(b.params_.grid_resolution_xy, 0.31);
    EXPECT_DOUBLE_EQ(b.params_.maximumComputationTime, 12.5);
}

TEST(Followups, ProgressCallbackIsCalled)
{
    mpp::PlannerInput in;
    in.ptgs            = buildTrs();
    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPoint2D{5.0, 1.0};
    in.worldBboxMin    = {-3, -6, -M_PI};
    in.worldBboxMax    = {9, 6, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.profiler_().enable(false);
    planner.progressCallbackCallPeriod_ = 0.0;
    int calls                           = 0;
    planner.progressCallback_           = [&](const mpp::ProgressCallbackData&)
    { calls++; };
    const auto po = planner.plan(in);
    EXPECT_TRUE(po.success);
    EXPECT_GT(calls, 0);
}
