/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Planning from a moving start (stateStart.vel != 0):
 *  - HolonomicBlend: the first edge starts with the start velocity, so a
 *    robot moving forward keeps moving forward at first, even toward a goal
 *    to its left.
 *  - DiffDrive_C (circular arcs): the start velocity does not change the
 *    plan, and planning still succeeds.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mrpt/config/CConfigFileMemory.h>

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

const char* kDiffDriveCfg = R"cfg(
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

mpp::PlannerOutput planFrom(
    const char* cfgText, const mrpt::math::TTwist2D& startVel,
    const mrpt::math::TPoint2D& goal)
{
    mrpt::config::CConfigFileMemory cfg(cfgText);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    in.stateStart.pose = {0, 0, 0};
    in.stateStart.vel  = startVel;
    in.stateGoal.state = goal;
    in.worldBboxMin    = {-6, -6, -M_PI};
    in.worldBboxMax    = {6, 6, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.profiler_().enable(false);
    planner.params_.maximumComputationTime = 30.0;
    return planner.plan(in);
}

// The first edge of the solution path:
const mpp::MoveEdgeSE2_TPS& firstEdge(const mpp::PlannerOutput& po)
{
    const auto [nodes, edges] = po.motionTree.backtrack_path(*po.goalNodeId);
    EXPECT_FALSE(edges.empty());
    return *edges.front();
}

// Position along an edge, `dt` seconds after its start:
mrpt::math::TPose2D poseAfter(
    const mpp::PlannerOutput& po, const mpp::MoveEdgeSE2_TPS& e, double dt)
{
    auto& ptg = *po.originalInput.ptgs.ptgs.at(e.ptgIndex);
    ptg.updateNavDynamicState(e.getPTGDynState());
    const auto step = static_cast<uint32_t>(dt / ptg.getPathStepDuration());
    return e.stateFrom.pose + ptg.getPathPose(e.ptgPathIndex, step);
}

}  // namespace

TEST(AstarStartVelocity, HolonomicKeepsInitialMotion)
{
    const mrpt::math::TPoint2D goal = {0.0, 3.0};

    const auto still  = planFrom(kHolonomicCfg, {0, 0, 0}, goal);
    const auto moving = planFrom(kHolonomicCfg, {0.8, 0, 0}, goal);
    ASSERT_TRUE(still.success);
    ASSERT_TRUE(moving.success);

    const auto& eMoving = firstEdge(moving);
    EXPECT_DOUBLE_EQ(eMoving.stateFrom.vel.vx, 0.8);
    EXPECT_NEAR(eMoving.getPTGDynState().curVelLocal.vx, 0.8, 1e-9);

    // 0.3 s after the start, the moving robot went on forward (+x) much more
    // than the one starting at rest:
    const double xMoving = poseAfter(moving, eMoving, 0.3).x;
    const double xStill  = poseAfter(still, firstEdge(still), 0.3).x;
    EXPECT_GT(xMoving, 0.15);
    EXPECT_GT(xMoving, xStill + 0.1);
}

TEST(AstarStartVelocity, DiffDriveIgnoresStartVelocity)
{
    const mrpt::math::TPoint2D goal = {4.0, 1.5};

    const auto still  = planFrom(kDiffDriveCfg, {0, 0, 0}, goal);
    const auto moving = planFrom(kDiffDriveCfg, {0.8, 0, 0.2}, goal);
    ASSERT_TRUE(still.success);
    ASSERT_TRUE(moving.success);
    EXPECT_DOUBLE_EQ(still.pathCost, moving.pathCost);
    EXPECT_EQ(still.numExpandedNodes, moving.numExpandedNodes);
}
