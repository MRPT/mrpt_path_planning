/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <selfdriving/algos/render_tree.h>
#include <selfdriving/algos/viz.h>

using namespace selfdriving;

static std::vector<mrpt::gui::CDisplayWindow3D::Ptr> nonmodal_wins;

void selfdriving::viz_nav_plan(
    const selfdriving::PlannerOutput&        plan,
    const selfdriving::VisualizationOptions& opts,
    const std::vector<CostEvaluator::Ptr>    costEvaluators)
{
    MRPT_START
    //
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
