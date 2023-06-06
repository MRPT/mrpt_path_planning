/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/render_tree.h>
#include <mpp/algos/render_vehicle.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mpp;

auto mpp::render_tree(
    const MotionPrimitivesTreeSE2& tree, const PlannerInput& pi,
    const RenderOptions& ro) -> std::shared_ptr<mrpt::opengl::CSetOfObjects>
{
    using mrpt::opengl::stock_objects::CornerXYZ;
    using mrpt::opengl::stock_objects::CornerXYZSimple;

    auto ret = mrpt::opengl::CSetOfObjects::Create();
    ret->setName("render_tree");
    auto& scene = *ret;

    const auto poseHeight = [&ro](const mrpt::poses::CPose3D& p) {
        if (ro.phi2z_scale == 0)
            return p;
        else
        {
            auto r = p;
            r.z(ro.phi2z_scale * r.yaw());
            return r;
        }
    };
    const auto poseHeightT = [&ro](const mrpt::math::TPose3D& p) {
        if (ro.phi2z_scale == 0)
            return p;
        else
        {
            auto r = p;
            r.z    = ro.phi2z_scale * r.yaw;
            return r;
        }
    };

    // Build a model of the vehicle shape:
    mrpt::opengl::CSetOfLines gl_veh_shape;

    double xyzcorners_scale;
    {
        gl_veh_shape.setLineWidth(ro.vehicle_line_width);
        gl_veh_shape.setColor_u8(ro.color_vehicle);
        auto res         = render_vehicle(pi.ptgs.robotShape, gl_veh_shape);
        xyzcorners_scale = res.maxVehicleShapeRadius * 0.20;
    }

    // Override with user scale?
    if (ro.xyzcorners_scale) xyzcorners_scale = *ro.xyzcorners_scale;

    // "ground"
    if (ro.ground_xy_grid_frequency.value_or(1.0) > 0)
    {
        double gridSpacing;
        if (ro.ground_xy_grid_frequency)
        {  // user value:
            gridSpacing = ro.ground_xy_grid_frequency.value();
        }
        else
        {
            const auto lx        = pi.worldBboxMax.x - pi.worldBboxMin.x;
            const auto ly        = pi.worldBboxMax.y - pi.worldBboxMin.y;
            const auto lSmallest = std::max(lx, ly);
            if (lSmallest > 0)
                gridSpacing = (lSmallest / 5.0) * 0.999;
            else
                gridSpacing = 1.0;
        }

        auto obj = mrpt::opengl::CGridPlaneXY::Create(
            pi.worldBboxMin.x, pi.worldBboxMax.x, pi.worldBboxMin.y,
            pi.worldBboxMax.y, 0 /*z*/, gridSpacing);
        obj->setColor_u8(ro.color_ground_xy_grid);
        scene.insert(obj);
    }

    // Original randomly-pick pose:
    if (ro.x_rand_pose)
    {
        auto obj = CornerXYZ(xyzcorners_scale * 1.0);
        obj->setName("X_rand");
        obj->enableShowName();
        obj->setPose(poseHeight(mrpt::poses::CPose3D(*ro.x_rand_pose)));
        scene.insert(obj);
    }

    // Nearest state pose:
    if (ro.x_nearest_pose)
    {
        auto obj = CornerXYZ(xyzcorners_scale * 1.0);
        obj->setName("X_near");
        obj->enableShowName();
        obj->setPose(poseHeight(mrpt::poses::CPose3D(*ro.x_nearest_pose)));
        scene.insert(obj);
    }

    // Determine the up-to-now best solution, so we can highlight the best path
    // so far:
    MotionPrimitivesTreeSE2::path_t best_path;

    if (ro.highlight_path_to_node_id &&
        tree.nodes().count(ro.highlight_path_to_node_id.value()))
    {
        auto [path, edges] = tree.backtrack_path(*ro.highlight_path_to_node_id);

        best_path = std::move(path);
    }

    // make list of nodes in the way of the best path:
    std::set<const MotionPrimitivesTreeSE2::edge_t*> edges_best_path,
        edges_best_path_decim;
    std::set<mrpt::graphs::TNodeID> bestPathNodeIDs;

    if (!best_path.empty())
    {
        const auto it_end = best_path.end();

        ASSERT_GT_(ro.draw_shape_decimation, 0);

        size_t       pathIdx   = 0;
        const size_t pathSteps = best_path.size();
        for (auto it = best_path.begin(); it != it_end; ++it, ++pathIdx)
        {
            bestPathNodeIDs.insert(it->nodeID_);

            if (it->nodeID_ == tree.root)
                continue;  // no edge-to-parent for the root!

            // Decimate the path (always keeping the first and last entry):
            const auto etp = &tree.edge_to_parent(it->nodeID_);

            edges_best_path.insert(etp);

            if (pathIdx == 0 || pathIdx + 1 == pathSteps ||
                (pathIdx % ro.draw_shape_decimation) == 0)
            {  // Decimated version:
                edges_best_path_decim.insert(etp);
            }
        }
    }

    // The starting pose vehicle shape must be inserted independently, because
    // the rest are edges and we draw the END pose of each edge:
    {
        auto vehShape  = mrpt::opengl::CSetOfLines::Create(gl_veh_shape);
        auto shapePose = mrpt::math::TPose3D(pi.stateStart.pose);
        shapePose.z += ro.vehicle_shape_z;
        vehShape->setPose(poseHeightT(shapePose));
        scene.insert(vehShape);
    }

    // Existing nodes & edges between them:
    for (const auto& idnode : tree.nodes())
    {
        const auto& node = idnode.second;

        mrpt::math::TPose2D poseParent;
        if (node.parentID_) poseParent = tree.nodes().at(*node.parentID_).pose;

        const mrpt::math::TPose2D& poseNode = node.pose;

        const MotionPrimitivesTreeSE2::edge_t* etp = nullptr;
        if (node.nodeID_ != tree.root) etp = &tree.edge_to_parent(node.nodeID_);

        const bool isLastNode = (idnode.first == tree.nodes().rbegin()->first);
        const bool isBestPath = etp && edges_best_path.count(etp) != 0;
        const bool isBestPathAndDrawShape =
            etp && edges_best_path_decim.count(etp) != 0;

        const bool drawTwistState =
            bestPathNodeIDs.count(node.nodeID_) && ro.draw_twist;

        // Draw children nodes:
        {
            const float corner_scale =
                xyzcorners_scale * (isLastNode ? 1.5f : 1.0f);

            bool cornerVisible = true;
            if (ro.width_normal_edge == 0)
            {
                // the user doesn't want to see regular edges.
                // don't show corners, neither:
                if (!isBestPath) cornerVisible = false;
            }

            if (cornerVisible)
            {
                auto obj = CornerXYZSimple(corner_scale);
                obj->setPose(poseHeight(mrpt::poses::CPose3D(poseNode)));
                scene.insert(obj);
            }

            // Insert vehicle shapes along optimal path:
            if (isBestPathAndDrawShape)
            {
                auto vehShape = mrpt::opengl::CSetOfLines::Create(gl_veh_shape);
                auto shapePose = mrpt::math::TPose3D(poseNode);
                shapePose.z += ro.vehicle_shape_z;
                vehShape->setPose(poseHeightT(shapePose));
                scene.insert(vehShape);
            }
            if (drawTwistState && cornerVisible)
            {
                // Draw twist:
                if (node.vel.vx != 0 || node.vel.vy != 0)
                {
                    auto glLinVel = mrpt::opengl::CArrow::Create();
                    glLinVel->setArrowEnds(
                        0, 0, 0, node.vel.vx * ro.linVelScale,
                        node.vel.vy * ro.linVelScale, .0);
                    glLinVel->setSmallRadius(ro.twistArrowsRadius);
                    glLinVel->setLargeRadius(ro.twistArrowsRadius * 1.5);
                    glLinVel->setColor_u8(0xff, 0x00, 0x00);
                    glLinVel->setLocation(
                        poseNode.x, poseNode.y, ro.phi2z_scale * poseNode.phi);
                    scene.insert(glLinVel);
                }

                if (node.vel.omega != 0)
                {
                    auto glAngVel = mrpt::opengl::CArrow::Create();
                    glAngVel->setArrowEnds(
                        0, 0, 0, 0, 0, node.vel.omega * ro.angVelScale);
                    glAngVel->setSmallRadius(ro.twistArrowsRadius);
                    glAngVel->setLargeRadius(ro.twistArrowsRadius * 1.5);
                    glAngVel->setColor_u8(0x00, 0xff, 0x00);
                    glAngVel->setLocation(
                        poseNode.x, poseNode.y, ro.phi2z_scale * poseNode.phi);
                    scene.insert(glAngVel);
                }
            }
        }

        // Draw actual PTG path between parent and children nodes:
        if (etp && !etp->interpolatedPath.empty())
        {
            // Create the path shape, in relative coords to the parent node:
            auto obj = mrpt::opengl::CSetOfLines::Create();
            obj->setPose(poseHeight(mrpt::poses::CPose3D(poseParent)));

            // Use stored interpolated path to avoid having to update PTG's
            // dynamic state and calling to ptg->renderPathAsSimpleLine():
            ASSERT_(!etp->interpolatedPath.empty());

            // dummy, just to allow the easy use of "strip" below:
            obj->appendLine(0, 0, 0, 0, 0, 0);
            const auto& ip = etp->interpolatedPath;
            for (const auto& timeRelPose : ip)
            {
                const auto& relPose = timeRelPose.second;
                obj->appendLineStrip(
                    relPose.x, relPose.y, ro.phi2z_scale * relPose.phi);
            }

            if (isLastNode && ro.highlight_last_added_edge)
            {
                // Last edge format:
                obj->setColor_u8(ro.color_last_edge);
                obj->setLineWidth(ro.width_last_edge);
            }
            else
            {
                // Normal format:
                obj->setColor_u8(ro.color_normal_edge);
                obj->setLineWidth(ro.width_normal_edge);
            }
            if (isBestPath)
            {
                obj->setColor_u8(ro.color_optimal_edge);
                obj->setLineWidth(ro.width_optimal_edge);
            }

            if (ro.showEdgeCosts && obj->getLineWidth() > 0)
            {
                auto objLb = mrpt::opengl::CText3D::Create();

                // Place the label by the midpoint of the path:
                const auto relPose = ip.rbegin()->second;

                objLb->setPose(obj->getPose() + relPose);
                objLb->setScale(ro.edgeCostLabelSize);
                objLb->setString(mrpt::format(
                    "c=%.02f d=%.02f", etp->cost,
                    etp->ptgDist != std::numeric_limits<double>::max()
                        ? etp->ptgDist
                        : .0));
                scene.insert(objLb);
            }

            if (obj->getLineWidth() > 0) scene.insert(obj);
        }
    }

    // The new node:
    if (ro.new_state)
    {
        auto obj = CornerXYZ(xyzcorners_scale * 1.2);
        obj->setName("X_new");
        obj->enableShowName();
        obj->setPose(poseHeight(mrpt::poses::CPose3D(*ro.new_state)));
        scene.insert(obj);
    }

    // Obstacles:
    if (ro.draw_obstacles)
    {
        for (const auto& os : pi.obstacles)
        {
            auto obj = mrpt::opengl::CPointCloud::Create();

            const auto obs = os->obstacles();

            obj->loadFromPointsMap(obs.get());

            obj->setPointSize(ro.point_size_obstacles);
            obj->setColor_u8(ro.color_obstacles);
            scene.insert(obj);
        }
    }

    // The current set of local obstacles:
    // Draw this AFTER the global map so it's visible:
    if (ro.draw_obstacles && ro.local_obs_from_nearest_pose &&
        ro.x_nearest_pose)
    {
        mrpt::opengl::CPointCloud::Ptr obj =
            mrpt::opengl::CPointCloud::Create();

        obj->loadFromPointsMap(ro.local_obs_from_nearest_pose.value());
        obj->setPose(*ro.x_nearest_pose);

        obj->setPointSize(ro.point_size_local_obstacles);
        obj->setColor_u8(ro.color_local_obstacles);
        scene.insert(obj);
    }

    // Start:
    {
        auto obj = CornerXYZ(xyzcorners_scale * 1.5);
        obj->setName("START");
        obj->enableShowName();
        obj->setColor_u8(ro.color_start);
        obj->setPose(poseHeightT(pi.stateStart.pose));
        scene.insert(obj);
    }

    // Target:
    if (pi.stateGoal.state.isPose())
    {
        auto obj = CornerXYZ(xyzcorners_scale * 1.5);
        obj->setName("GOAL");
        obj->enableShowName();
        obj->setColor_u8(ro.color_goal);
        obj->setPose(poseHeightT(pi.stateGoal.state.pose()));
        scene.insert(obj);
    }
    else if (pi.stateGoal.state.isPoint())
    {
        auto obj = mrpt::opengl::CDisk::Create();
        obj->setDiskRadius(xyzcorners_scale * 1.5, xyzcorners_scale * 1.25);
        obj->setName("GOAL");
        obj->enableShowName();
        obj->setColor_u8(ro.color_goal);
        obj->setLocation(pi.stateGoal.state.point());
        scene.insert(obj);
    }
    else
    {
        THROW_EXCEPTION("Unknown type for goal.state");
    }

    // Log msg:
    if (!ro.log_msg.empty())
    {
        auto gl_txt =
            mrpt::opengl::CText3D::Create(ro.log_msg, "sans", ro.log_msg_scale);
        gl_txt->setLocation(ro.log_msg_position);
        scene.insert(gl_txt);
    }

    return ret;
}
