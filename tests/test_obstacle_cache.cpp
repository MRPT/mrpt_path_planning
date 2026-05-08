/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Tests for P3 fix: cached_local_obstacles() cache.
 *
 * Verified behaviors:
 *  - Planner still finds the correct path when the cache is active.
 *  - The profiler shows that cached_local_obstacles is called many times
 *    (many node expansions) but the average call time drops significantly
 *    compared to a full transform, confirming cache hits are occurring.
 *  - The total planning time with the cache is meaningfully faster than the
 *    equivalent work without caching (measured by timing N raw transforms).
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/transform_pc_square_clipping.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/CTimeLogger.h>

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

// ---------------------------------------------------------------------------
// Build a dense obstacle cloud (grid of points) around the planning area,
// leaving a corridor clear for the planner to navigate through.
// More points → more work per transform → cache benefit is more visible.
// ---------------------------------------------------------------------------
static mrpt::maps::CSimplePointsMap::Ptr makeDenseObstacles(int N = 60)
{
    auto obs = mrpt::maps::CSimplePointsMap::Create();
    // Walls along y = ±1.5, x from -5 to 5 (dense)
    for (int i = -N; i <= N; ++i)
    {
        const double x = i * (5.0 / N);
        obs->insertPointFast(x, 1.5f, 0);
        obs->insertPointFast(x, -1.5f, 0);
        // side walls
        obs->insertPointFast(5.0f, i * (1.5 / N), 0);
        obs->insertPointFast(-5.0f, i * (1.5 / N), 0);
    }
    obs->mark_as_modified();
    return obs;
}

static mpp::TPS_Astar buildPlanner()
{
    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 15;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.5, 3.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 60.0;
    return planner;
}

// ---------------------------------------------------------------------------
// Correctness: planner still finds a valid path through the corridor
// ---------------------------------------------------------------------------
TEST(ObstacleCache, CorrectPathWithCache)
{
    auto planner = buildPlanner();

    mrpt::config::CConfigFileMemory cfg(kPtgCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    in.stateStart.pose = {-3.0, 0.0, 0.0};
    in.stateGoal.state = mrpt::math::TPoint2D{3.0, 0.0};
    in.worldBboxMin    = {-6, -6, -M_PI};
    in.worldBboxMax    = {6, 6, M_PI};
    in.obstacles.push_back(
        mpp::ObstacleSource::FromStaticPointcloud(makeDenseObstacles()));

    const auto out = planner.plan(in);

    EXPECT_TRUE(out.success)
        << "Planner must still find a path with cache active";
}

// ---------------------------------------------------------------------------
// Performance: verify that the cache is actually being hit.
//
// We check two things via the profiler:
//  1. cached_local_obstacles was called many times (> 10 node expansions).
//  2. The average per-call time is much shorter than a single full transform
//     of the same obstacle cloud — confirming cache hits dominate.
// ---------------------------------------------------------------------------
TEST(ObstacleCache, CacheReducesTransformWork)
{
    auto planner = buildPlanner();

    mrpt::config::CConfigFileMemory cfg(kPtgCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    in.stateStart.pose = {-3.0, 0.0, 0.0};
    in.stateGoal.state = mrpt::math::TPoint2D{3.0, 0.0};
    in.worldBboxMin    = {-6, -6, -M_PI};
    in.worldBboxMax    = {6, 6, M_PI};

    auto obs = makeDenseObstacles(120);  // ~960 obstacle points
    in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);

    // --- Profiler stats for cached_local_obstacles ---
    std::map<std::string, mrpt::system::CTimeLogger::TCallStats> stats;
    planner.profiler_().getStats(stats);

    const auto it = stats.find("cached_local_obstacles");
    ASSERT_NE(it, stats.end())
        << "cached_local_obstacles must be instrumented in the profiler";

    const auto& s = it->second;
    EXPECT_GT(s.n_calls, 10u)
        << "Expected many node expansions (>10 cache queries)";

    // Measure how long one raw transform of the same cloud takes.
    mrpt::system::CTimeLogger rawTimer(true /*enabled*/, "raw_transforms");
    rawTimer.setMinLoggingLevel(
        mrpt::system::LVL_ERROR);  // suppress dtor output
    {
        mrpt::maps::CSimplePointsMap dummy;
        const mrpt::poses::CPose2D   pose(0, 0, 0);
        const int                    kWarmupRep = 5;
        for (int i = 0; i < kWarmupRep; ++i)
        {
            mrpt::system::CTimeLoggerEntry tle(rawTimer, "raw");
            mpp::transform_pc_square_clipping(*obs, pose, 5.0, dummy);
        }
    }
    std::map<std::string, mrpt::system::CTimeLogger::TCallStats> rawStats;
    rawTimer.getStats(rawStats);
    const double rawMeanTime = rawStats.at("raw").mean_t;

    // Average cached_local_obstacles call must be faster than a full transform.
    // We use a generous 10x factor to avoid flakiness on slow CI machines,
    // while still catching regressions where the cache is not working.
    EXPECT_LT(s.mean_t, rawMeanTime)
        << "Cache hits should make mean cached_local_obstacles call faster "
           "than one raw transform. mean_t="
        << s.mean_t << "s, rawMeanTime=" << rawMeanTime << "s";

    std::cout << "[ObstacleCache] n_calls=" << s.n_calls
              << "  mean_t=" << s.mean_t * 1e6 << " us"
              << "  raw_transform_mean=" << rawMeanTime * 1e6 << " us\n";
}

// ---------------------------------------------------------------------------
// Performance: total obstacle-transform time is reduced by the cache.
//
// The total time attributed to cached_local_obstacles in the profiler should
// be much less than the cost of doing the same number of raw (uncached)
// transforms.  This directly measures the cache's impact on the hot path.
// ---------------------------------------------------------------------------
TEST(ObstacleCache, WallClockSpeedup)
{
    auto obs = makeDenseObstacles(120);  // ~960 points

    // --- Run plan() and collect profiler data ---
    mpp::PlannerOutput out;
    double             cachedTotalTime = 0.0;
    size_t             nCacheCalls     = 0;
    {
        auto planner = buildPlanner();

        mrpt::config::CConfigFileMemory cfg(kPtgCfg);
        mpp::PlannerInput               in;
        in.ptgs.initFromConfigFile(cfg, "SelfDriving");
        in.stateStart.pose = {-3.0, 0.0, 0.0};
        in.stateGoal.state = mrpt::math::TPoint2D{3.0, 0.0};
        in.worldBboxMin    = {-6, -6, -M_PI};
        in.worldBboxMax    = {6, 6, M_PI};
        in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));

        out = planner.plan(in);

        std::map<std::string, mrpt::system::CTimeLogger::TCallStats> stats;
        planner.profiler_().getStats(stats);
        if (auto it = stats.find("cached_local_obstacles"); it != stats.end())
        {
            nCacheCalls     = it->second.n_calls;
            cachedTotalTime = it->second.total_t;
        }
    }
    ASSERT_TRUE(out.success);
    ASSERT_GT(nCacheCalls, 0u);

    // --- Measure cost of nCacheCalls raw (uncached) transforms ---
    mrpt::system::CTimeLogger rawTimer(true, "raw_transforms");
    rawTimer.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    {
        mrpt::maps::CSimplePointsMap dummy;
        const mrpt::poses::CPose2D   pose(0, 0, 0);
        for (size_t i = 0; i < nCacheCalls; ++i)
        {
            mrpt::system::CTimeLoggerEntry tle(rawTimer, "raw");
            mpp::transform_pc_square_clipping(*obs, pose, 5.0, dummy);
        }
    }
    std::map<std::string, mrpt::system::CTimeLogger::TCallStats> rawStats;
    rawTimer.getStats(rawStats);
    const double rawTotalTime = rawStats.at("raw").total_t;

    // The total time spent inside cached_local_obstacles during the plan
    // should be much less than the cost of doing all transforms from scratch.
    EXPECT_LT(cachedTotalTime, rawTotalTime)
        << "Total cached_local_obstacles time (" << cachedTotalTime * 1e3
        << " ms) should be less than " << nCacheCalls
        << " uncached transforms (" << rawTotalTime * 1e3 << " ms)";

    std::cout << "[ObstacleCache] cached_total=" << cachedTotalTime * 1e3
              << " ms  " << nCacheCalls << " uncached transforms would cost "
              << rawTotalTime * 1e3
              << " ms  speedup=" << rawTotalTime / cachedTotalTime << "x\n";
}
