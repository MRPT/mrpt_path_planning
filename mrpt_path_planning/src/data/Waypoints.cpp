/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mpp/data/Waypoints.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSetOfObjects.h>

using namespace mpp;

// Waypoint  ==========
Waypoint::Waypoint(
    double target_x, double target_y, double allowed_distance_, bool allowSkip_,
    std::optional<double> target_heading_, double speed_ratio_)
    : target(target_x, target_y),
      targetHeading(target_heading_),
      allowedDistance(allowed_distance_),
      speedRatio(speed_ratio_),
      allowSkip(allowSkip_)
{
    // Backwards-compatibility:
    if (targetHeading.has_value() && *targetHeading == Waypoint::INVALID_NUM)
        targetHeading.reset();
}

bool Waypoint::isValid() const
{
    return (target.x != INVALID_NUM) && (target.y != INVALID_NUM) &&
           (allowedDistance != INVALID_NUM);
}

mrpt::math::TPose2D Waypoint::targetAsPose() const
{
    return {
        target.x, target.y,
        targetHeading.has_value() ? targetHeading.value() : .0};
}

std::string Waypoint::getAsText() const
{
    std::string s;
    if (target.x != INVALID_NUM && target.y != INVALID_NUM)
        s += mrpt::format("target=(%8.03f,%8.03f) ", target.x, target.y);
    else
        s += "target=(**Coordinates not set!!**) ";

    if (targetHeading.has_value())
        s += mrpt::format(
            "phi=%8.03f deg ", mrpt::RAD2DEG(targetHeading.value()));
    else
        s += " (heading: any) ";

    if (allowedDistance != INVALID_NUM)
        s += mrpt::format("allowed_dist=%8.03f ", allowedDistance);
    else
        s += " (**allowed_distance not set!!**) ";

    s += (allowSkip ? " allowSkip: YES" : " allowSkip: NO ");

    s += mrpt::format(" speed_ratio: %.01f", speedRatio);
    return s;
}

mrpt::containers::yaml Waypoint::asYAML() const
{
    mrpt::containers::yaml d = mrpt::containers::yaml::Map();

    d["target"] = mrpt::containers::yaml::Sequence({target.x, target.y});
    d["target"].node().printInShortFormat = true;

    if (targetHeading) d["targetHeading"] = mrpt::RAD2DEG(*targetHeading);

    d["targetFrameId"]   = targetFrameId;
    d["allowedDistance"] = allowedDistance;
    d["speedRatio"]      = speedRatio;
    d["allowSkip"]       = allowSkip;
    d["preferNotToSkip"] = preferNotToSkip;

    return d;
}

Waypoint Waypoint::FromYAML(const mrpt::containers::yaml& d)
{
    Waypoint wp;

    const auto s = d["target"];
    ASSERT_(s.isSequence());

    wp.target = {
        s.asSequence()[0].as<double>(), s.asSequence()[1].as<double>()};

    if (d.has("targetHeading"))
        wp.targetHeading = mrpt::DEG2RAD(d["targetHeading"].as<double>());

    wp.targetFrameId   = d["targetFrameId"].as<std::string>();
    wp.allowedDistance = d["allowedDistance"].as<double>();
    wp.speedRatio      = d["speedRatio"].as<double>();
    wp.allowSkip       = d["allowSkip"].as<bool>();
    wp.preferNotToSkip = d["preferNotToSkip"].as<bool>();

    return wp;
}

// WaypointSequence ==========

// Gets navigation params as a human-readable format:
std::string WaypointSequence::getAsText() const
{
    std::string s;
    s += mrpt::format(
        "List of %u waypoints:\n", static_cast<unsigned int>(waypoints.size()));
    unsigned int i = 0;
    for (const auto& wp : waypoints)
    {
        s += mrpt::format(" #%3u: ", i++);
        s += wp.getAsText();
        s += "\n";
    }
    return s;
}

// WaypointStatus ==========
std::string WaypointStatus::getAsText() const
{
    std::string s = Waypoint::getAsText();
    s += mrpt::format(" reached=%s", (reached ? "YES" : "NO "));
    return s;
}

// WaypointStatusSequence ======
std::string WaypointStatusSequence::getAsText() const
{
    std::string s;
    s += mrpt::format(
        "Status for %u waypoints:\n",
        static_cast<unsigned int>(waypoints.size()));
    unsigned int i = 0;
    for (const auto& wp : waypoints)
    {
        s += mrpt::format(" #%3u: ", i++);
        s += wp.getAsText();
        s += "\n";
    }
    s += mrpt::format(
        " final_goal_reached:%s  waypoint_index_current_goal=%s\n",
        (final_goal_reached ? "YES" : "NO "),
        waypoint_index_current_goal
            ? std::to_string(*waypoint_index_current_goal).c_str()
            : "(Not started)");
    return s;
}

WaypointsRenderingParams::WaypointsRenderingParams()
    : color_regular(mrpt::img::TColor(0x00, 0x00, 0xff)),
      color_current_goal(mrpt::img::TColor(0xff, 0x00, 0x20)),
      color_reached(mrpt::img::TColor(0x00, 0x00, 0xc0, 0xd0))

{
}

void WaypointSequence::getAsOpenglVisualization(
    mrpt::opengl::CSetOfObjects&    obj,
    const WaypointsRenderingParams& params) const
{
    obj.clear();
    unsigned int idx = 0;
    for (const auto& p : waypoints)
    {
        auto gl_pt = mrpt::opengl::CDisk::Create(
            p.allowSkip ? params.outter_radius
                        : params.outter_radius_non_skippable,
            p.allowSkip ? params.inner_radius
                        : params.inner_radius_non_skippable,
            15);
        gl_pt->setLocation(p.target.x, p.target.y, 0.01);
        gl_pt->setColor_u8(params.color_regular);
        if (params.show_labels)
        {
            gl_pt->setName(mrpt::format("WayPt #%2u", idx));
            gl_pt->enableShowName(true);
        }
        obj.insert(gl_pt);

        if (p.targetHeading.has_value())
        {
            auto o = mrpt::opengl::CArrow::Create(
                0, 0, 0, params.heading_arrow_len, 0.0f, 0.0f);
            o->setPose(mrpt::poses::CPose3D(
                p.target.x, p.target.y, 0.02, p.targetHeading.value(), 0, 0));
            obj.insert(o);
        }
        ++idx;
    }
}

void WaypointStatusSequence::getAsOpenglVisualization(
    mrpt::opengl::CSetOfObjects&    obj,
    const WaypointsRenderingParams& params) const
{
    obj.clear();
    {
        unsigned int idx = 0;
        for (const auto& p : waypoints)
        {
            const bool is_cur_goal = (int(idx) == waypoint_index_current_goal);

            mrpt::opengl::CDisk::Ptr gl_pt = mrpt::opengl::CDisk::Create(
                p.reached ? params.outter_radius_reached
                          : (p.allowSkip ? params.outter_radius
                                         : params.outter_radius_non_skippable),
                p.reached ? params.inner_radius_reached
                          : (p.allowSkip ? params.inner_radius
                                         : params.inner_radius_non_skippable),
                15);
            gl_pt->setLocation(p.target.x, p.target.y, 0.01);
            if (params.show_labels)
            {
                gl_pt->setName(mrpt::format(
                    "WayPt #%2u Reach:%s", idx, p.reached ? "YES" : "NO"));
                gl_pt->enableShowName(true);
            }
            gl_pt->setColor_u8(
                is_cur_goal ? params.color_current_goal
                            : (p.reached ? params.color_reached
                                         : params.color_regular));
            obj.insert(gl_pt);

            if (p.targetHeading.has_value())
            {
                auto o = mrpt::opengl::CArrow::Create(
                    0, 0, 0, params.heading_arrow_len, 0.0f, 0.0f);
                o->setPose(mrpt::poses::CPose3D(
                    p.target.x, p.target.y, 0.02, p.targetHeading.value(), 0,
                    0));
                obj.insert(o);
            }
            ++idx;
        }
    }
}

mrpt::containers::yaml WaypointSequence::asYAML() const
{
    auto n = mrpt::containers::yaml::Sequence();

    for (const auto& wp : waypoints) n.asSequence().emplace_back(wp.asYAML());

    return n;
}

WaypointSequence WaypointSequence::FromYAML(const mrpt::containers::yaml& d)
{
    WaypointSequence ws;
    ASSERT_(d.isSequence());

    for (const auto& e : d.asSequence())
        ws.waypoints.emplace_back(Waypoint::FromYAML(e));

    return ws;
}
