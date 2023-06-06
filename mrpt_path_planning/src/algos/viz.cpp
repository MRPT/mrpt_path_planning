/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/render_tree.h>
#include <mpp/algos/viz.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/system/CTicTac.h>

#include <thread>

using namespace mpp;

static std::vector<mrpt::gui::CDisplayWindow3D::Ptr> nonmodal_wins;

void mpp::viz_nav_plan(
    const mpp::PlannerOutput& plan, const mpp::VisualizationOptions& opts,
    const std::vector<CostEvaluator::Ptr> costEvaluators)
{
    MRPT_START
    auto win = mrpt::gui::CDisplayWindow3D::Create("Path plan viz", 800, 600);

    mrpt::opengl::COpenGLScene::Ptr scene;

    // Build opengl scene:
    {
        mrpt::gui::CDisplayWindow3DLocker dwl(*win, scene);

        auto glTree = render_tree(
            plan.motionTree, plan.originalInput, opts.renderOptions);
        scene->insert(glTree);

        for (const auto& ce : costEvaluators)
        {
            if (!ce) continue;
            scene->insert(ce->get_visualization());
        }
    }

    // Camera:
    win->setCameraAzimuthDeg(-90);
    win->setCameraElevationDeg(90);
    win->setCameraProjective(false);

    // Render:
    win->updateWindow();

    if (opts.gui_modal)
    {
        // Wait:
        win->waitForKey();
    }
    else
    {
        nonmodal_wins.push_back(win);
    }

    MRPT_END
}

void mpp::viz_nav_plan_animation(
    const PlannerOutput& plan, const mpp::trajectory_t& traj,
    const RenderOptions&                  opts,
    const std::vector<CostEvaluator::Ptr> costEvaluators)
{
    MRPT_START

    ASSERT_(!traj.empty());

    // Path interpolation:
    mrpt::poses::CPose2DInterpolator trajPath;
    for (const auto& kv : traj)
    {
        // NOTE: These are "fake" timestamps, but it's ok (they are at the
        // beginning of UNIX epoch).
        const auto t = mrpt::Clock::fromDouble(kv.first);

        trajPath.insert(t, kv.second.state.pose);
    }
    trajPath.setInterpolationMethod(
        mrpt::poses::TInterpolatorMethod::imLinearSlerp);

    // Create UI:
    auto win = mrpt::gui::CDisplayWindow3D::Create("Path plan viz", 800, 600);

    mrpt::opengl::COpenGLScene::Ptr scene;

    auto glVehFrame = mrpt::opengl::CSetOfObjects::Create();
    auto glVeh      = mrpt::opengl::CSetOfObjects::Create();

    auto glRobotShape = mrpt::opengl::CSetOfLines::Create();
    plan.originalInput.ptgs.ptgs.front()->add_robotShape_to_setOfLines(
        *glRobotShape);
    glRobotShape->setColor_u8(0xff, 0x00, 0x00, 0xff);  // RGB+A
    glVeh->insert(glRobotShape);

    glVehFrame->insert(glVeh);

    // The path is referenced to the path planning "start pose", account for it:
    glVehFrame->setPose(plan.originalInput.stateStart.pose);

    // Build opengl scene:
    {
        mrpt::gui::CDisplayWindow3DLocker dwl(*win, scene);

        auto glTree = render_tree(plan.motionTree, plan.originalInput, opts);
        scene->insert(glTree);
        scene->insert(glVehFrame);

        for (const auto& ce : costEvaluators)
        {
            if (!ce) continue;
            scene->insert(ce->get_visualization());
        }
    }

    // Camera:
    win->setCameraAzimuthDeg(-90);
    win->setCameraElevationDeg(90);
    win->setCameraProjective(false);

    // Render:
    win->updateWindow();

    // Wait for window close and run animation in the meanwhile:
    mrpt::system::CTicTac stopWatch;

    while (win->isOpen())
    {
        // find pose at this moment in time:
        const double        t = stopWatch.Tac();
        mrpt::math::TPose2D vehPose;
        bool                validInterp = false;
        trajPath.interpolate(mrpt::Clock::fromDouble(t), vehPose, validInterp);
        if (validInterp)
        {
            mrpt::gui::CDisplayWindow3DLocker dwl(*win, scene);
            glVeh->setPose(vehPose);
        }
        // time wrap:
        if (t > mrpt::Clock::toDouble(trajPath.rbegin()->first))
            stopWatch.Tic();  // reset to t=0

        // refresh:
        win->updateWindow();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    MRPT_END
}
