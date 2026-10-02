/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for the robot description in TrajectoriesAndRobotShape.
 *
 * Verified behaviors:
 *  - RobotModel_min_turning_radius is loaded (default 0).
 *  - Loading C-PTGs turning tighter than the vehicle can is an error.
 *  - Footprint comparison (as used to cross-check nodes' configurations).
 */

#include <gtest/gtest.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/data/robot_shape_sampling.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/format.h>

namespace
{
// C-PTG with turning radius v_max/w_max = 1.0 / (30 deg/s) = 1.91 m
std::string config(const std::string& extra)
{
    return R"cfg(
[SelfDriving]
PTG_COUNT = 1
PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 2.0
PTG0_num_paths   = 11
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 30.0
PTG0_K           = +1.0
RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg" + extra +
           "\n";
}
}  // namespace

TEST(RobotDescription, DefaultMinTurningRadius)
{
    mrpt::config::CConfigFileMemory cfg(config(""));
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    EXPECT_DOUBLE_EQ(trs.minTurningRadius, 0.0);
}

TEST(RobotDescription, ConsistentMinTurningRadius)
{
    mrpt::config::CConfigFileMemory cfg(
        config("RobotModel_min_turning_radius = 1.5"));
    mpp::TrajectoriesAndRobotShape trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    EXPECT_DOUBLE_EQ(trs.minTurningRadius, 1.5);
}

TEST(RobotDescription, PtgTighterThanVehicleIsAnError)
{
    mrpt::config::CConfigFileMemory cfg(
        config("RobotModel_min_turning_radius = 2.5"));
    mpp::TrajectoriesAndRobotShape trs;
    EXPECT_ANY_THROW(trs.initFromConfigFile(cfg, "SelfDriving"));
}

TEST(RobotDescription, SameRobotShape)
{
    mrpt::math::TPolygon2D a;
    a.emplace_back(-0.2, -0.2);
    a.emplace_back(0.3, -0.2);
    a.emplace_back(0.3, 0.2);
    a.emplace_back(-0.2, 0.2);

    // Same shape, other starting vertex:
    mrpt::math::TPolygon2D b;
    b.emplace_back(0.3, 0.2);
    b.emplace_back(-0.2, 0.2);
    b.emplace_back(-0.2, -0.2);
    b.emplace_back(0.3, -0.2);
    EXPECT_TRUE(mpp::sameRobotShape(a, b));

    // Longer:
    auto c = a;
    c[1].x = 0.5;
    c[2].x = 0.5;
    EXPECT_FALSE(mpp::sameRobotShape(a, c));

    // Circles:
    const auto c1 = mpp::robotShapeAsPolygon(mpp::robot_radius_t{0.5});
    const auto c2 = mpp::robotShapeAsPolygon(mpp::robot_radius_t{0.6});
    EXPECT_EQ(c1.size(), 16U);
    EXPECT_TRUE(mpp::sameRobotShape(c1, c1));
    EXPECT_FALSE(mpp::sameRobotShape(c1, c2));
    EXPECT_FALSE(mpp::sameRobotShape(a, c1));
}
