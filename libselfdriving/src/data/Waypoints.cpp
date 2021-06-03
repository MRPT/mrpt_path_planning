/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <selfdriving/data/Waypoints.h>

using namespace selfdriving;

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

// WaypointSequence ==========
WaypointSequence::WaypointSequence() = default;
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
WaypointStatus& WaypointStatus::operator=(const Waypoint& wp)
{
    Waypoint::operator=(wp);
    return *this;
}
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
        " final_goal_reached:%s  waypoint_index_current_goal=%d\n",
        (final_goal_reached ? "YES" : "NO "), waypoint_index_current_goal);
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

void WaypointSequence::save(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
    const unsigned int N = waypoints.size();
    c.write(s, "waypoint_count", N);

    const int NP = 27;  // name padding

    for (unsigned int i = 0; i < N; i++)
    {
        const auto& wp = waypoints[i];

        c.write(
            s, mrpt::format("wp%03u_allowed_distance", i), wp.allowedDistance,
            NP);
        c.write(s, mrpt::format("wp%03u_allowSkip", i), wp.allowSkip, NP);
        c.write(s, mrpt::format("wp%03u_target_x", i), wp.target.x, NP);
        c.write(s, mrpt::format("wp%03u_target_y", i), wp.target.y, NP);
        c.write(
            s, mrpt::format("wp%03u_target_frame_id", i), wp.targetFrameId, NP);
        if (wp.targetHeading.has_value())
            c.write(
                s, mrpt::format("wp%03u_target_heading", i), *wp.targetHeading,
                NP);
        c.write(s, mrpt::format("wp%03u_speed_ratio", i), wp.speedRatio, NP);
    }
}

void WaypointSequence::load(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
    this->clear();

    const unsigned int N = c.read_int(s, "waypoint_count", 0, true);
    waypoints.resize(N);

    for (unsigned int i = 0; i < N; i++)
    {
        auto& wp = waypoints[i];

        wp.allowedDistance = c.read_double(
            s, mrpt::format("wp%03u_allowed_distance", i), 0, true);
        wp.allowSkip =
            c.read_bool(s, mrpt::format("wp%03u_allowSkip", i), true, true);
        wp.target.x =
            c.read_double(s, mrpt::format("wp%03u_target_x", i), 0, true);
        wp.target.y =
            c.read_double(s, mrpt::format("wp%03u_target_y", i), 0, true);
        wp.targetFrameId = c.read_string(
            s, mrpt::format("wp%03u_target_frame_id", i), "map", false);

        const auto sectHeading = mrpt::format("wp%03u_target_heading", i);
        if (c.keyExists(s, sectHeading))
            wp.targetHeading =
                c.read_double(s, sectHeading, Waypoint::INVALID_NUM);

        wp.speedRatio =
            c.read_double(s, mrpt::format("wp%03u_speed_ratio", i), 1.0, false);
    }
}
