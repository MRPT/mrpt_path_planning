/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/viz.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>

using namespace selfdrive;

static std::vector<mrpt::gui::CDisplayWindow3D::Ptr> nonmodal_wins;

static mrpt::opengl::CRenderizable::Ptr getRobotShapeViz(const RobotShape& r)
{
    if (r.robot_shape.empty())
    {
#if 0
        return mrpt::opengl::CCylinder::Create(
            r.robot_radius, r.robot_radius, 0.10f /*height*/, 20 /*slices*/,
            2 /*stacks*/);
#else
        auto obj = mrpt::opengl::CCylinder::Create(
            r.robot_radius, r.robot_radius, 0.05f /*height*/, 20 /*slices*/,
            2 /*stacks*/);
        obj->setHasBases(false, false);
        return obj;
#endif
    }
    else
    {
        THROW_EXCEPTION("TO DO!");
    }
}

void selfdrive::viz_nav_plan(
    const selfdrive::NavPlan& plan, const selfdrive::NavPlanRenderOptions& opts)
{
    MRPT_START
    //
    auto win = mrpt::gui::CDisplayWindow3D::Create("Path plan viz", 800, 600);

    mrpt::opengl::COpenGLScene::Ptr scene;

    // Build opengl scene:
    {
        mrpt::gui::CDisplayWindow3DLocker dwl(*win, scene);

        scene->insert(mrpt::opengl::CGridPlaneXY::Create(
            -10.f, 10.0f, -10.f, 10.f, 0.0f, 1.0f));

        // Goal pose:
        {
            auto gl_group = mrpt::opengl::CSetOfObjects::Create();
            auto gl_obj =
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 6.0f);
            gl_group->insert(gl_obj);
            gl_group->insert(getRobotShapeViz(plan.original_input.robot_shape));

            gl_group->setLocation(
                plan.original_input.state_goal.pose.x,
                plan.original_input.state_goal.pose.y, 0);

            scene->insert(gl_group);
        }
        // Intermediary poses:
        int stepCount = -1;
        for (const auto& p : plan.actions)
        {
            auto gl_group = mrpt::opengl::CSetOfObjects::Create();
            auto gl_obj =
                mrpt::opengl::stock_objects::CornerXYZSimple(0.25f, 3.0f);
            gl_group->insert(gl_obj);

            if (stepCount == -1 || ++stepCount >= opts.show_robot_shape_every_N)
            {
                stepCount = 0;
                gl_group->insert(
                    getRobotShapeViz(plan.original_input.robot_shape));
            }

            gl_group->setPose(p.state_from.pose);
            scene->insert(gl_group);
        }

        // Obstacles:
        auto obs = plan.original_input.obstacles->obstacles();
        if (obs)
        {
            obs->renderOptions.color.R    = 1;
            obs->renderOptions.color.G    = 0;
            obs->renderOptions.color.B    = 0;
            obs->renderOptions.point_size = 5.0f;

            auto gl_obs = mrpt::opengl::CSetOfObjects::Create();
            obs->getAs3DObject(gl_obs);
            scene->insert(gl_obs);
        }
    }

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
