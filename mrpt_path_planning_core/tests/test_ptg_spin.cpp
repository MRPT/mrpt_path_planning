/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for the rotation in place PTG (mpp::ptg::DiffDrive_Spin).
 *
 * Verified behaviors:
 *  - Trajectories are pure rotations at the maximum angular speed, ending
 *    exactly at their target rotation (+-pi for the extreme ones), in the
 *    minimum time.
 *  - The TP-Space distance is the rotated angle times turningRadiusReference.
 *  - rotationToTrajectoryStep() finds the step reaching any rotation of up to
 *    pi, and rejects larger or null rotations.
 *  - Config: only the rotation speed and grid resolution are required.
 *  - A vehicle with a minimum turning radius rejects this PTG.
 *  - Obstacles beyond the footprint circumscribed circle never block a
 *    rotation; obstacles within it block both directions before half a turn.
 *  - Clones are independent and identical.
 *  - refine_trajectory() keeps the direction of half turns, whose end pose
 *    is the same both ways.
 */

#include <gtest/gtest.h>
#include <mpp/algos/edge_interpolated_path.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/ptgs/DiffDrive_Spin.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/math/wrap2pi.h>

#include <cmath>
#include <memory>
#include <string>

namespace
{
constexpr double kWmaxDeg = 60.0;

std::string spinConfig(
    const std::string& extraPtgKeys = "", double minTurningRadius = 0)
{
    return std::string(R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0

PTG_COUNT = 1

PTG0_Type        = mpp::ptg::DiffDrive_Spin
PTG0_resolution  = 0.05
PTG0_w_max_dps   = 60.0
)cfg") + extraPtgKeys +
           R"cfg(
RobotModel_shape2D_xs = -0.30 0.30 0.30 -0.30
RobotModel_shape2D_ys = -0.20 -0.20 0.20 0.20
RobotModel_min_turning_radius = )cfg" +
           std::to_string(minTurningRadius) + "\n";
}

std::shared_ptr<mpp::ptg::DiffDrive_Spin> loadSpin(
    mpp::TrajectoriesAndRobotShape& trs, const std::string& cfgText)
{
    mrpt::config::CConfigFileMemory cfg(cfgText);
    trs.initFromConfigFile(cfg, "SelfDriving");
    EXPECT_EQ(trs.ptgs.size(), 1U);
    auto spin =
        std::dynamic_pointer_cast<mpp::ptg::DiffDrive_Spin>(trs.ptgs.at(0));
    EXPECT_TRUE(spin);
    return spin;
}
}  // namespace

TEST(PtgSpin, DefaultsAndConfig)
{
    mpp::TrajectoriesAndRobotShape trs;
    const auto                     spin = loadSpin(trs, spinConfig());
    ASSERT_TRUE(spin);
    EXPECT_EQ(spin->getPathCount(), 2U);
    EXPECT_EQ(spin->getMaxLinVel(), 0.0);
    EXPECT_NEAR(spin->getMaxAngVel(), mrpt::DEG2RAD(kWmaxDeg), 1e-9);
    EXPECT_NEAR(spin->getTrajectoryRotation(0), -M_PI, 1e-9);
    EXPECT_NEAR(spin->getTrajectoryRotation(1), M_PI, 1e-9);
    EXPECT_FALSE(spin->inverseMap_WS2TP(0.0, 0.0).has_value());
    EXPECT_FALSE(spin->inverseMap_WS2TP(0.5, 0.2).has_value());
}

TEST(PtgSpin, PureRotationsAtMaxSpeed)
{
    mpp::TrajectoriesAndRobotShape trs;
    const auto spin = loadSpin(trs, spinConfig("PTG0_num_paths = 7\n"));
    ASSERT_TRUE(spin);

    const double      wMax = mrpt::DEG2RAD(kWmaxDeg);
    const double      dt   = spin->getPathStepDuration();
    const mpp::ptg_t& ptg  = *spin;  // (getPathTwist() is public there)
    for (uint16_t k = 0; k < spin->getPathCount(); k++)
    {
        const double rot    = spin->getTrajectoryRotation(k);
        const size_t nSteps = spin->getPathStepCount(k);
        ASSERT_GE(nSteps, 2U);
        for (uint32_t n = 0; n < nSteps; n++)
        {
            const auto p  = spin->getPathPose(k, n);
            const auto tw = ptg.getPathTwist(k, n);
            EXPECT_EQ(p.x, 0.0);
            EXPECT_EQ(p.y, 0.0);
            EXPECT_EQ(tw.vx, 0.0);
            EXPECT_EQ(tw.vy, 0.0);
            EXPECT_LE(std::abs(tw.omega), wMax + 1e-6);
            EXPECT_NEAR(
                spin->getPathDist(k, n),
                std::abs(p.phi) * spin->getRefDistance() / (1.01 * M_PI), 1e-4);
        }
        // Ends exactly at the target rotation, in the minimum time:
        EXPECT_NEAR(spin->getPathPose(k, nSteps - 1).phi, rot, 1e-4) << k;
        EXPECT_NEAR((nSteps - 1) * dt, std::abs(rot) / wMax, 3 * dt) << k;
    }
    // Odd count: the middle trajectory does not move.
    EXPECT_EQ(spin->getTrajectoryRotation(3), 0.0);
}

TEST(PtgSpin, RotationToTrajectoryStep)
{
    mpp::TrajectoriesAndRobotShape trs;
    const auto                     spin = loadSpin(trs, spinConfig());
    ASSERT_TRUE(spin);
    const double halfStepAngle =
        0.5 * mrpt::DEG2RAD(kWmaxDeg) * spin->getPathStepDuration();

    for (double deg = -180; deg <= 180; deg += 7.5)
    {
        const double angle = mrpt::DEG2RAD(deg);
        const auto   ks    = spin->rotationToTrajectoryStep(angle);
        if (deg == 0)
        {
            EXPECT_FALSE(ks.has_value());
            continue;
        }
        ASSERT_TRUE(ks.has_value()) << deg;
        const auto p = spin->getPathPose(ks->first, ks->second);
        EXPECT_NEAR(p.phi, angle, halfStepAngle + 1e-5) << deg;
        EXPECT_EQ(p.x, 0.0);
        EXPECT_EQ(p.y, 0.0);
    }
    EXPECT_FALSE(spin->rotationToTrajectoryStep(M_PI + 0.01).has_value());
    EXPECT_FALSE(spin->rotationToTrajectoryStep(-M_PI - 0.01).has_value());
    EXPECT_FALSE(spin->rotationToTrajectoryStep(0.1 * halfStepAngle));
}

TEST(PtgSpin, RejectedWithMinTurningRadius)
{
    mrpt::config::CConfigFileMemory cfg(spinConfig("", 0.5));
    mpp::TrajectoriesAndRobotShape  trs;
    EXPECT_ANY_THROW(trs.initFromConfigFile(cfg, "SelfDriving"));
}

TEST(PtgSpin, ObstaclesVsFootprintCircle)
{
    mpp::TrajectoriesAndRobotShape trs;
    const auto                     spin = loadSpin(trs, spinConfig());
    ASSERT_TRUE(spin);
    const double fullTurn = spin->getPathDist(1, spin->getPathStepCount(1) - 1);

    // Circumscribed radius: hypot(0.3, 0.2) = 0.36 m. Points beyond it (and
    // beyond the conservative grid margin) never block a rotation:
    for (double a = 0; a < 2 * M_PI; a += 0.3)
    {
        const double        r = 0.36 + 0.15;
        std::vector<double> tp;
        spin->initTPObstacles(tp);
        spin->updateTPObstacle(r * std::cos(a), r * std::sin(a), tp);
        for (const double d : tp) { EXPECT_GT(d, fullTurn) << a; }
    }

    // A point beside the robot, outside the footprint but inside the circle,
    // blocks both rotations before half a turn:
    std::vector<double> tp;
    spin->initTPObstacles(tp);
    spin->updateTPObstacle(0.0, 0.30, tp);
    ASSERT_EQ(tp.size(), 2U);
    for (const double d : tp) { EXPECT_LT(d, 0.5 * fullTurn); }
}

TEST(PtgSpin, CloneIsIndependent)
{
    mpp::TrajectoriesAndRobotShape trs;
    const auto                     spin = loadSpin(trs, spinConfig());
    ASSERT_TRUE(spin);
    const auto copy = trs.clone();
    ASSERT_EQ(copy.ptgs.size(), 1U);
    const auto* c =
        dynamic_cast<const mpp::ptg::DiffDrive_Spin*>(copy.ptgs.at(0).get());
    ASSERT_NE(c, nullptr);
    EXPECT_NE(c, spin.get());
    EXPECT_EQ(c->getPathStepCount(1), spin->getPathStepCount(1));
    const auto ks = c->rotationToTrajectoryStep(1.0);
    ASSERT_TRUE(ks.has_value());
    EXPECT_NEAR(c->getPathPose(ks->first, ks->second).phi, 1.0, 1e-3);
}

TEST(PtgSpin, RefineKeepsHalfTurnDirection)
{
    mpp::TrajectoriesAndRobotShape trs;
    const auto                     spin = loadSpin(trs, spinConfig());
    ASSERT_TRUE(spin);

    for (const uint16_t k : {uint16_t(0), uint16_t(1)})
    {
        for (const double phi0 : {0.0, 1.0, -2.5})
        {
            const uint32_t step =
                static_cast<uint32_t>(spin->getPathStepCount(k) - 1);
            mpp::MotionPrimitivesTreeSE2::node_t n0;
            mpp::MotionPrimitivesTreeSE2::node_t n1;
            n0.pose = {1.0, 2.0, phi0};
            n1.pose = n0.pose + spin->getPathPose(k, step);

            mpp::MoveEdgeSE2_TPS e;
            e.ptgIndex       = 0;
            e.ptgPathIndex   = k;
            e.ptgStepIndex   = step;
            e.ptgDist        = spin->getPathDist(k, step);
            e.stateFrom.pose = n0.pose;
            e.stateTo.pose   = n1.pose;
            mpp::edge_interpolated_path(
                e, trs, spin->getPathPose(k, step), step, 5);

            // The end node pose, slightly moved (as by lattice rounding):
            for (const double dphi : {0.0, 1e-4, -1e-4})
            {
                auto n1b = n1;
                n1b.pose.phi += dphi;
                auto edges = std::vector<mpp::MoveEdgeSE2_TPS>{e};
                mpp::refine_trajectory({n0, n1b}, edges, trs);
                EXPECT_EQ(edges.at(0).ptgPathIndex, k)
                    << "phi0=" << phi0 << " dphi=" << dphi;
            }
        }
    }
}
