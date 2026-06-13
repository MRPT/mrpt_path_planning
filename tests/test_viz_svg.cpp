/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/** Unit tests for the SVG plan exporter (plan_to_svg / save_plan_to_svg). */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/viz_svg.h>
#include <mpp/data/PlannerInput.h>
#include <mrpt/config/CConfigFileMemory.h>

#include <cstdio>
#include <fstream>

static const char* kCfg = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0
PTG_COUNT = 1
PTG0_Type        = mpp::ptg::HolonomicBlend
PTG0_refDistance = 5.0
PTG0_num_paths   = 31
PTG0_T_ramp_max  = 1.0
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_expr_V      = V_MAX * trimmable_speed
PTG0_expr_W      = W_MAX * trimmable_speed
PTG0_expr_T_ramp = T_ramp_max
RobotModel_circular_shape_radius = 0.20
)cfg";

static mpp::PlannerOutput planSimple()
{
    mrpt::config::CConfigFileMemory cfg(kCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPoint2D{3.0, 1.0};
    in.worldBboxMin    = {-2, -2, -M_PI};
    in.worldBboxMax    = {5, 4, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.maximumComputationTime = 20.0;
    return planner.plan(in);
}

TEST(VizSvg, ProducesWellFormedSvg)
{
    const auto out = planSimple();
    ASSERT_TRUE(out.success);

    const std::string svg = mpp::plan_to_svg(out);

    EXPECT_NE(svg.find("<?xml"), std::string::npos);
    EXPECT_NE(svg.find("<svg"), std::string::npos);
    EXPECT_NE(svg.find("</svg>"), std::string::npos);
    // a bold solution path polyline must be present:
    EXPECT_NE(svg.find("polyline"), std::string::npos);
    // start/goal markers (circles) present:
    EXPECT_NE(svg.find("<circle"), std::string::npos);
}

TEST(VizSvg, SaveToFile)
{
    const auto        out = planSimple();
    const std::string fn  = "test_viz_svg_output.svg";
    ASSERT_TRUE(mpp::save_plan_to_svg(out, fn));

    std::ifstream f(fn);
    ASSERT_TRUE(f.is_open());
    std::string first;
    std::getline(f, first);
    EXPECT_NE(first.find("<?xml"), std::string::npos);
    f.close();
    std::remove(fn.c_str());
}

TEST(VizSvg, OptionsTogglesReduceOutput)
{
    const auto out = planSimple();

    mpp::SvgExportOptions full;  // everything on
    mpp::SvgExportOptions minimal;
    minimal.draw_obstacles    = false;
    minimal.draw_tree         = false;
    minimal.draw_robot_shapes = false;

    EXPECT_GT(
        mpp::plan_to_svg(out, full).size(),
        mpp::plan_to_svg(out, minimal).size());
}
