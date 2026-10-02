/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * TPS_Astar from a start pose closer to an obstacle than the PTG clearance.
 *
 * Verified behaviors:
 *  - A robot that stopped within the clearance of a wall can still plan a
 *    path away from it.
 *  - A start pose actually overlapping an obstacle has no solution.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>

namespace
{
// Forward + backward C-PTGs keeping 0.15 m of clearance; 0.5x0.36 m robot.
const char* kCfg = R"cfg(
[SelfDriving]
PTG_COUNT = 2

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.05
PTG0_refDistance = 3.0
PTG0_num_paths   = 31
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0
PTG0_clearance   = 0.15

PTG1_Type        = mpp::ptg::DiffDrive_C
PTG1_resolution  = 0.05
PTG1_refDistance = 3.0
PTG1_num_paths   = 31
PTG1_v_max_mps   = 1.0
PTG1_w_max_dps   = 60.0
PTG1_K           = -1.0
PTG1_clearance   = 0.15

RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

// Plans from the origin (heading +x) to (4,0) with a wall along y = wallY.
bool planWithWallAt(double wallY)
{
    mrpt::config::CConfigFileMemory cfg(kCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    auto obs = mrpt::maps::CSimplePointsMap::Create();
    for (double x = -2.0; x <= 6.0; x += 0.05)
    {
        obs->insertPoint(x, wallY, 0);
    }
    in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));

    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPose2D(4.0, -1.0, 0.0);
    in.worldBboxMin    = {-3, -3, -M_PI};
    in.worldBboxMax    = {7, 3, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 15;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.0, 2.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 20.0;

    return planner.plan(in).success;
}
}  // namespace

TEST(TPS_Astar, StartWithinClearance)
{
    // Wall 0.05 m from the robot side (clearance is 0.15 m):
    EXPECT_TRUE(planWithWallAt(0.18 + 0.05));
    // Wall far enough:
    EXPECT_TRUE(planWithWallAt(1.0));
}

TEST(TPS_Astar, StartInCollision)
{
    // The wall crosses the robot footprint:
    EXPECT_FALSE(planWithWallAt(0.10));
}
