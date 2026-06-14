/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/viz_svg.h>
#include <mrpt/poses/CPose2D.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <variant>
#include <vector>

using namespace mpp;

namespace
{
// World->image transform (SVG y points down, so the world y axis is flipped).
struct Frame
{
    double xmin, ymin, ymax, scale, margin;
    double tx(double x) const { return margin + (x - xmin) * scale; }
    double ty(double y) const { return margin + (ymax - y) * scale; }
};

std::string fmt(double v)
{
    std::ostringstream s;
    s << std::fixed;
    s.precision(2);
    s << v;
    return s.str();
}

// Append the robot footprint (polygon or circle) at a global pose.
void appendRobotShape(
    std::ostream& os, const RobotShape& shape, const mrpt::math::TPose2D& pose,
    const Frame& fr, const SvgExportOptions& o)
{
    if (std::holds_alternative<mrpt::math::TPolygon2D>(shape))
    {
        const auto& poly = std::get<mrpt::math::TPolygon2D>(shape);
        const mrpt::poses::CPose2D p(pose);
        os << "<polygon points=\"";
        for (const auto& v : poly)
        {
            double gx = 0, gy = 0;
            p.composePoint(v.x, v.y, gx, gy);
            os << fmt(fr.tx(gx)) << "," << fmt(fr.ty(gy)) << " ";
        }
        os << "\" fill=\"" << o.color_robot_fill << "\" stroke=\""
           << o.color_robot << "\" stroke-width=\"" << o.stroke_robot_px
           << "\"/>\n";
    }
    else if (std::holds_alternative<robot_radius_t>(shape))
    {
        const double r = std::get<robot_radius_t>(shape);
        os << "<circle cx=\"" << fmt(fr.tx(pose.x)) << "\" cy=\""
           << fmt(fr.ty(pose.y)) << "\" r=\"" << fmt(r * fr.scale)
           << "\" fill=\"" << o.color_robot_fill << "\" stroke=\""
           << o.color_robot << "\" stroke-width=\"" << o.stroke_robot_px
           << "\"/>\n";
    }
}

// Global poses sampled along an edge (interpolatedPath if present, else the
// straight from->to segment as a fallback, e.g. for deferred-interpolation
// tree edges).
std::vector<mrpt::math::TPose2D> edgePoses(const MoveEdgeSE2_TPS& e)
{
    std::vector<mrpt::math::TPose2D> out;
    if (e.interpolatedPath.size() >= 2)
    {
        for (const auto& [t, relPose] : e.interpolatedPath)
        {
            (void)t;
            out.push_back(e.stateFrom.pose + relPose);
        }
    }
    else
    {
        out.push_back(e.stateFrom.pose);
        out.push_back(e.stateTo.pose);
    }
    return out;
}

void polyline(
    std::ostream& os, const std::vector<mrpt::math::TPose2D>& pts,
    const Frame& fr, const std::string& color, double width)
{
    if (pts.size() < 2) return;
    os << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\""
       << width << "\" points=\"";
    for (const auto& p : pts)
        os << fmt(fr.tx(p.x)) << "," << fmt(fr.ty(p.y)) << " ";
    os << "\"/>\n";
}

// drawHeading=false marks a R(2)-only pose (heading unconstrained, "ANY"):
// instead of a directional tick, draw a dashed ring around the dot.
void marker(
    std::ostream& os, const mrpt::math::TPose2D& p, const Frame& fr,
    const std::string& color, bool drawHeading = true)
{
    const double cx = fr.tx(p.x);
    const double cy = fr.ty(p.y);
    os << "<circle cx=\"" << fmt(cx) << "\" cy=\"" << fmt(cy)
       << "\" r=\"5\" fill=\"" << color << "\"/>\n";
    if (drawHeading)
    {
        // heading tick (note: image y is flipped, so use -sin for screen):
        const double hx = cx + 12.0 * std::cos(p.phi);
        const double hy = cy - 12.0 * std::sin(p.phi);
        os << "<line x1=\"" << fmt(cx) << "\" y1=\"" << fmt(cy) << "\" x2=\""
           << fmt(hx) << "\" y2=\"" << fmt(hy) << "\" stroke=\"" << color
           << "\" stroke-width=\"2\"/>\n";
    }
    else
    {
        os << "<circle cx=\"" << fmt(cx) << "\" cy=\"" << fmt(cy)
           << "\" r=\"10\" fill=\"none\" stroke=\"" << color
           << "\" stroke-width=\"1.5\" stroke-dasharray=\"3,2\"/>\n";
    }
}
}  // namespace

std::string mpp::plan_to_svg(
    const PlannerOutput& plan, const SvgExportOptions& o)
{
    const auto& pi = plan.originalInput;

    // Drawing extent: union of the world bbox, obstacles, and all tree node
    // poses, so that legitimate path "bulges" outside the planning bbox (e.g.
    // an arc routing around a finite wall through free space) are NOT clipped
    // off-canvas. This makes such behavior visible for debugging.
    double     xmin = pi.worldBboxMin.x, xmax = pi.worldBboxMax.x;
    double     ymin = pi.worldBboxMin.y, ymax = pi.worldBboxMax.y;
    const auto grow = [&](double x, double y)
    {
        xmin = std::min(xmin, x);
        xmax = std::max(xmax, x);
        ymin = std::min(ymin, y);
        ymax = std::max(ymax, y);
    };
    for (const auto& kv : plan.motionTree.nodes())
        grow(kv.second.pose.x, kv.second.pose.y);
    for (const auto& os_src : pi.obstacles)
    {
        if (!os_src || !os_src->obstacles()) continue;
        const auto& xs = os_src->obstacles()->getPointsBufferRef_x();
        const auto& ys = os_src->obstacles()->getPointsBufferRef_y();
        for (size_t i = 0; i < os_src->obstacles()->size(); i++)
            grow(xs[i], ys[i]);
    }
    // a little world-space padding so footprints near the edge are not cut:
    const double pad = 0.5;
    xmin -= pad;
    xmax += pad;
    ymin -= pad;
    ymax += pad;

    Frame fr;
    fr.xmin             = xmin;
    fr.ymin             = ymin;
    fr.ymax             = ymax;
    fr.margin           = o.margin_px;
    const double worldW = xmax - xmin;
    const double worldH = ymax - ymin;
    fr.scale =
        (worldW > 0) ? (o.image_width_px - 2 * o.margin_px) / worldW : 1.0;
    const double imgW = o.image_width_px;
    const double imgH = worldH * fr.scale + 2 * o.margin_px;

    std::ostringstream os;
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << fmt(imgW)
       << "\" height=\"" << fmt(imgH) << "\" viewBox=\"0 0 " << fmt(imgW) << " "
       << fmt(imgH) << "\">\n";
    os << "<rect width=\"100%\" height=\"100%\" fill=\"" << o.color_background
       << "\"/>\n";

    // World bounding box:
    if (o.draw_bbox)
    {
        os << "<rect x=\"" << fmt(fr.tx(pi.worldBboxMin.x)) << "\" y=\""
           << fmt(fr.ty(pi.worldBboxMax.y)) << "\" width=\""
           << fmt(worldW * fr.scale) << "\" height=\"" << fmt(worldH * fr.scale)
           << "\" fill=\"none\" stroke=\"" << o.color_bbox
           << "\" stroke-width=\"1\"/>\n";
    }

    // Obstacles:
    if (o.draw_obstacles)
    {
        os << "<g fill=\"" << o.color_obstacles << "\">\n";
        const size_t dec = std::max<size_t>(1, o.obstacle_decimation);
        for (const auto& os_src : pi.obstacles)
        {
            if (!os_src) continue;
            const auto obs = os_src->obstacles();
            if (!obs) continue;
            const auto& xs = obs->getPointsBufferRef_x();
            const auto& ys = obs->getPointsBufferRef_y();
            for (size_t i = 0; i < obs->size(); i += dec)
            {
                os << "<circle cx=\"" << fmt(fr.tx(xs[i])) << "\" cy=\""
                   << fmt(fr.ty(ys[i])) << "\" r=\"" << o.obstacle_radius_px
                   << "\"/>\n";
            }
        }
        os << "</g>\n";
    }

    // Full motion tree (faint):
    if (o.draw_tree)
    {
        os << "<g>\n";
        for (const auto& kv : plan.motionTree.edges_to_children)
        {
            for (const auto& entry : kv.second)
            {
                polyline(
                    os, edgePoses(entry.data), fr, o.color_tree,
                    o.stroke_tree_px);
            }
        }
        os << "</g>\n";
    }

    // Solution / best path (bold) + robot shapes along it:
    const auto bestId = plan.bestNodeId ? plan.bestNodeId : plan.goalNodeId;
    if (o.draw_path && bestId.has_value() &&
        plan.motionTree.nodes().count(bestId.value()))
    {
        const auto [nodes, edges] =
            plan.motionTree.backtrack_path(bestId.value());
        (void)nodes;

        std::vector<mrpt::math::TPose2D> full;
        for (const auto* e : edges)
        {
            if (!e) continue;
            const auto ps = edgePoses(*e);
            full.insert(full.end(), ps.begin(), ps.end());
        }
        polyline(os, full, fr, o.color_path, o.stroke_path_px);

        if (o.draw_robot_shapes && !full.empty())
        {
            os << "<g>\n";
            if (o.robot_shape_decimation > 0)
            {
                for (size_t i = 0; i < full.size();
                     i += o.robot_shape_decimation)
                    appendRobotShape(os, pi.ptgs.robotShape, full[i], fr, o);
            }
            appendRobotShape(os, pi.ptgs.robotShape, full.front(), fr, o);
            appendRobotShape(os, pi.ptgs.robotShape, full.back(), fr, o);
            os << "</g>\n";
        }
    }

    // Start & goal markers:
    if (o.draw_start_goal)
    {
        marker(os, pi.stateStart.pose, fr, o.color_start);
        // R(2) goals (point only) have an unconstrained ("ANY") heading: do
        // not draw a fake heading tick for them, draw a dashed ring instead.
        const bool goalHasHeading = !pi.stateGoal.state.isPoint();
        marker(
            os, pi.stateGoal.asSE2KinState().pose, fr, o.color_goal,
            goalHasHeading);
    }

    // Scale bar (1 m) + status text:
    if (o.draw_scalebar)
    {
        const double y = imgH - 8;
        os << "<line x1=\"" << fmt(o.margin_px) << "\" y1=\"" << fmt(y)
           << "\" x2=\"" << fmt(o.margin_px + fr.scale) << "\" y2=\"" << fmt(y)
           << "\" stroke=\"#000000\" stroke-width=\"2\"/>\n";
        os << "<text x=\"" << fmt(o.margin_px) << "\" y=\"" << fmt(y - 4)
           << "\" font-size=\"11\" font-family=\"sans-serif\">1 m</text>\n";
    }
    os << "<text x=\"" << fmt(o.margin_px) << "\" y=\"16\" font-size=\"12\" "
       << "font-family=\"sans-serif\">" << (plan.success ? "success" : "FAILED")
       << "</text>\n";

    os << "</svg>\n";
    return os.str();
}

bool mpp::save_plan_to_svg(
    const PlannerOutput& plan, const std::string& filename,
    const SvgExportOptions& o)
{
    std::ofstream f(filename);
    if (!f.is_open()) return false;
    f << plan_to_svg(plan, o);
    return f.good();
}
