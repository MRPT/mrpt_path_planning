/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/viz_svg.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>

#include <algorithm>
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

std::string fmt(double v, int precision = 2)
{
    std::ostringstream s;
    s << std::fixed;
    s.precision(precision);
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

using timed_pose_t = std::pair<double, mrpt::math::TPose2D>;

// Global poses sampled along an edge, with times relative to the edge start
// (interpolatedPath if present, else the straight from->to segment as a
// fallback, e.g. for deferred-interpolation tree edges).
std::vector<timed_pose_t> edgeTimedPoses(const MoveEdgeSE2_TPS& e)
{
    std::vector<timed_pose_t> out;
    if (e.interpolatedPath.size() >= 2)
    {
        for (const auto& [t, relPose] : e.interpolatedPath)
        {
            out.emplace_back(t, e.stateFrom.pose + relPose);
        }
    }
    else
    {
        out.emplace_back(0.0, e.stateFrom.pose);
        out.emplace_back(e.estimatedExecTime, e.stateTo.pose);
    }
    return out;
}

std::vector<mrpt::math::TPose2D> edgePoses(const MoveEdgeSE2_TPS& e)
{
    std::vector<mrpt::math::TPose2D> out;
    for (const auto& tp : edgeTimedPoses(e)) { out.push_back(tp.second); }
    return out;
}

// Robot footprint centered at the image origin, heading along +x, so it can
// be placed with an SVG transform.
void appendRobotShapeAtOrigin(
    std::ostream& os, const RobotShape& shape, const Frame& fr,
    const SvgExportOptions& o)
{
    const std::string style = "fill=\"" + o.color_robot_animated_fill +
                              "\" stroke=\"" + o.color_robot_animated +
                              "\" stroke-width=\"" + fmt(o.stroke_robot_px) +
                              "\"";
    double headingLength = 0;
    if (std::holds_alternative<mrpt::math::TPolygon2D>(shape))
    {
        const auto& poly = std::get<mrpt::math::TPolygon2D>(shape);
        os << "<polygon points=\"";
        for (const auto& v : poly)
        {
            // image y points down:
            os << fmt(v.x * fr.scale) << "," << fmt(-v.y * fr.scale) << " ";
            headingLength = std::max(headingLength, v.x * fr.scale);
        }
        os << "\" " << style << "/>\n";
    }
    else if (std::holds_alternative<robot_radius_t>(shape))
    {
        headingLength = std::get<robot_radius_t>(shape) * fr.scale;
        os << "<circle cx=\"0\" cy=\"0\" r=\"" << fmt(headingLength) << "\" "
           << style << "/>\n";
    }
    // Heading tick, so rotations are visible for any shape:
    os << "<line x1=\"0\" y1=\"0\" x2=\"" << fmt(headingLength)
       << "\" y2=\"0\" stroke=\"" << o.color_robot_animated
       << "\" stroke-width=\"" << fmt(o.stroke_robot_px) << "\"/>\n";
}

// A robot footprint moving along `keyframes` (global poses with absolute
// times), looping forever. Translation and rotation are animated in two
// nested groups, since each <animateTransform> handles one transform type.
void appendAnimatedRobot(
    std::ostream& os, const RobotShape& shape,
    const std::vector<timed_pose_t>& keyframes, const Frame& fr,
    const SvgExportOptions& o)
{
    if (keyframes.size() < 2) { return; }
    const double speed = o.animation_speed > 0 ? o.animation_speed : 1.0;
    const double t0    = keyframes.front().first;
    const double motionDuration = (keyframes.back().first - t0) / speed;
    const double totalDuration =
        motionDuration + std::max(0.0, o.animation_pause_at_end);
    if (totalDuration <= 0) { return; }

    std::ostringstream keyTimes;
    std::ostringstream translations;
    std::ostringstream rotations;

    // Unwrapped heading, so the interpolation never spins the long way round:
    double phi     = keyframes.front().second.phi;
    double prevPhi = phi;
    for (size_t i = 0; i < keyframes.size(); i++)
    {
        const auto& [t, p] = keyframes[i];
        phi += mrpt::math::wrapToPi(p.phi - prevPhi);
        prevPhi = p.phi;

        const double kt =
            std::clamp((t - t0) / speed / totalDuration, 0.0, 1.0);
        const bool isLastKey =
            (i + 1 == keyframes.size()) && motionDuration >= totalDuration;
        const char* sep = (i == 0) ? "" : ";";
        keyTimes << sep << (isLastKey ? std::string("1") : fmt(kt, 5));
        translations << sep << fmt(fr.tx(p.x)) << " " << fmt(fr.ty(p.y));
        // world angles are CCW, image rotations are CW (y axis down):
        rotations << sep << fmt(-mrpt::RAD2DEG(phi));
    }
    std::string lastTranslation;
    std::string lastRotation;
    {
        const auto& p   = keyframes.back().second;
        lastTranslation = fmt(fr.tx(p.x)) + " " + fmt(fr.ty(p.y));
        lastRotation    = fmt(-mrpt::RAD2DEG(phi));
    }
    if (motionDuration < totalDuration)
    {
        // Hold the final pose until the loop restarts:
        keyTimes << ";1";
        translations << ";" << lastTranslation;
        rotations << ";" << lastRotation;
    }

    const auto&       p0 = keyframes.front().second;
    const std::string timing =
        "dur=\"" + fmt(totalDuration, 3) + "s\" keyTimes=\"" + keyTimes.str() +
        "\" calcMode=\"linear\" repeatCount=\"indefinite\"";

    // The static transforms are what non-animating viewers show (start pose):
    os << "<g transform=\"translate(" << fmt(fr.tx(p0.x)) << " "
       << fmt(fr.ty(p0.y)) << ")\">\n";
    os << "<animateTransform attributeName=\"transform\" type=\"translate\" "
       << "values=\"" << translations.str() << "\" " << timing << "/>\n";
    os << "<g transform=\"rotate(" << fmt(-mrpt::RAD2DEG(p0.phi)) << ")\">\n";
    os << "<animateTransform attributeName=\"transform\" type=\"rotate\" "
       << "values=\"" << rotations.str() << "\" " << timing << "/>\n";
    appendRobotShapeAtOrigin(os, shape, fr, o);
    os << "</g>\n</g>\n";
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
        const size_t treeDec = o.tree_decimation > 0 ? o.tree_decimation : 1;
        size_t       edgeIdx = 0;
        for (const auto& kv : plan.motionTree.edges_to_children)
        {
            for (const auto& entry : kv.second)
            {
                if ((edgeIdx++ % treeDec) != 0) continue;
                polyline(
                    os, edgePoses(entry.data), fr, o.color_tree,
                    o.stroke_tree_px);
            }
        }
        os << "</g>\n";
    }

    // Solution / best path (bold) + robot shapes along it:
    std::vector<timed_pose_t> pathKeyframes;  // for the animation
    const auto bestId = plan.bestNodeId ? plan.bestNodeId : plan.goalNodeId;
    if (o.draw_path && bestId.has_value() &&
        plan.motionTree.nodes().count(bestId.value()))
    {
        const auto [nodes, edges] =
            plan.motionTree.backtrack_path(bestId.value());
        (void)nodes;

        std::vector<mrpt::math::TPose2D> full;
        double                           edgeStartTime = 0;
        for (const auto* e : edges)
        {
            if (!e) continue;
            const auto ps = edgeTimedPoses(*e);
            for (const auto& [t, p] : ps)
            {
                full.push_back(p);
                pathKeyframes.emplace_back(edgeStartTime + t, p);
            }
            edgeStartTime += ps.back().first;
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

    if (o.animate_robot)
    {
        appendAnimatedRobot(os, pi.ptgs.robotShape, pathKeyframes, fr, o);
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
    if (o.draw_status_text)
    {
        os << "<text x=\"" << fmt(o.margin_px)
           << "\" y=\"16\" font-size=\"12\" font-family=\"sans-serif\">"
           << (plan.success ? "success" : "FAILED") << "</text>\n";
    }

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
