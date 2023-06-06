/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/system/datetime.h>

#include <any>
#include <optional>
#include <string>
#include <vector>

namespace mpp
{
using waypoint_idx_t = std::size_t;

/** A single waypoint within WaypointSequence. */
struct Waypoint
{
    /** Ctor with default values */
    Waypoint() = default;

    Waypoint(
        double target_x, double target_y, double allowed_distance,
        bool                  allow_skip      = true,
        std::optional<double> target_heading_ = std::nullopt,
        double                speed_ratio_    = 1.0);

    /** [Mandatory] Coordinates of desired target location
     * (world/global coordinates).
     * \sa target_heading, targetAsPose()
     */
    mrpt::math::TPoint2D target{INVALID_NUM, INVALID_NUM};

    /** Set to the optional desired orientation [radians] of the robot at this
     * waypoint. (Default: none=any heading).
     *
     * Some navigator implementations may ignore this preferred heading,
     * read the docs of each implementation to find it out.
     *
     * \sa targetAsPose()
     */
    std::optional<double> targetHeading;

    /** Returns the target as a SE(2) pose, with its correct heading if
     * targetHeading is defined, or heading=0 otherwise.
     */
    mrpt::math::TPose2D targetAsPose() const;

    /** (Default="map") Frame ID in which target is given. Optional, use
     * only for submapping applications. */
    std::string targetFrameId = "map";

    /** [Mandatory] How close should the robot get to this waypoint for it to be
     * considered reached. */
    double allowedDistance = INVALID_NUM;

    /** (Default=1.0) Desired robot speed at the target, as a ratio of the full
     * robot speed. That is: speed_ratio=1 means that the user wants the robot
     * to navigate to the target and smoothly continue to the next one when
     * reached. speed_ratio=0 on the other hand means that the robot should
     * approach this waypoint slowing down and end up totally stopped.
     */
    double speedRatio = 1.0;

    /** [Default=true] Whether it is allowed to the navigator to proceed to a
     * more advanced waypoint
     * in the sequence if it determines that it is easier to skip this one
     * (e.g. it seems blocked by dynamic obstacles).
     * This value is ignored for the last waypoint in a sequence, since it is
     * always considered to be the ultimate goal and hence not subject to be
     * skipped.
     *
     * \sa preferNotToSkip
     */
    bool allowSkip = true;

    /**
     * This modifies the behavior of TWaypoint::allowSkip according
     * to:
     *
     * \verbatim
     * +------------+-----------------+---------------------------+
     * | allowSkip  | preferNotToSkip |    Waypoint obstructed    |
     * |            |                 |      with obstacles?      |
     * |            |                 +------------+--------------+
     * |            |                 |     Yes    |      No      |
     * +------------+-----------------+------------+--------------+
     * |    false   |      false      |  Trigger   | Pass through |
     * |  (default) +-----------------+ rnav error |   waypoint   |
     * |            |       true      |            |              |
     * +------------+-----------------+------------+--------------+
     * |    true    |  false(default) |   Skipped  |    Skipped   |
     * +------------+-----------------+            +--------------+
     * |    true    |       true      |            |  Not skipped |
     * +------------+-----------------+------------+--------------+
     * \endverbatim
     *
     */
    bool preferNotToSkip = false;

    /** Check whether all the minimum mandatory fields have been filled by the
     * user. */
    bool isValid() const;

    /** get in human-readable format */
    std::string getAsText() const;

    /** Save waypoint as YAML */
    mrpt::containers::yaml asYAML() const;

    /** Load waypoint from YAML  */
    static Waypoint FromYAML(const mrpt::containers::yaml& d);

    /** The default value of fields (used to detect non-set values) */
    static constexpr int INVALID_NUM{-100000};
};

/** used in getAsOpenglVisualization() */
struct WaypointsRenderingParams
{
    WaypointsRenderingParams();

    double outter_radius{.3}, inner_radius{.2};
    double outter_radius_non_skippable{.3}, inner_radius_non_skippable{.0};
    double outter_radius_reached{.2}, inner_radius_reached{.1};
    double heading_arrow_len{1.0};
    mrpt::img::TColor color_regular, color_current_goal, color_reached;
    bool              show_labels{true};
};

/** The struct for requesting navigation requests for a sequence of waypoints.
 * Users can directly fill in the list of waypoints manipulating the public
 * field `waypoints`.
 */
struct WaypointSequence
{
    std::vector<Waypoint> waypoints;

    /** Ctor with default values */
    WaypointSequence() = default;

    void clear() { waypoints.clear(); }

    /** Gets navigation params as a human-readable format */
    std::string getAsText() const;

    /** Renders the sequence of waypoints (previous contents of `obj` are
     * cleared) */
    void getAsOpenglVisualization(
        mrpt::opengl::CSetOfObjects&    obj,
        const WaypointsRenderingParams& params = {}) const;

    /** Save waypoints as YAML */
    mrpt::containers::yaml asYAML() const;

    /** Load waypoints from YAML  */
    static WaypointSequence FromYAML(const mrpt::containers::yaml& d);
};

/** A waypoint with an execution status. \ingroup nav_reactive */
struct WaypointStatus : public Waypoint
{
    WaypointStatus() = default;

    /** Whether this waypoint has been reached already (to within the allowed
     * distance as per user specifications) or skipped. */
    bool reached{false};

    /** If `reached==true` this boolean tells whether the waypoint was
     * physically reached (false) or marked as reached because it was skipped
     * (true). */
    bool skipped{false};

    /** Timestamp of when this waypoint was reached. (Default=INVALID_TIMESTAMP
     * means not reached so far) */
    mrpt::system::TTimeStamp timestamp_reach = INVALID_TIMESTAMP;

    /** (Initialized to 0 automatically) How many times this waypoint has been
     * seen as "reachable" before it being the current active waypoint. */
    int counter_seen_reachable{0};

    /** Gets navigation params as a human-readable format */
    std::string getAsText() const;
};

/** The struct for querying the status of waypoints navigation.
 */
struct WaypointStatusSequence
{
    WaypointStatusSequence() = default;

    /** Waypoints parameters and status (reached, skipped, etc.) */
    std::vector<WaypointStatus> waypoints;

    /** Extracts a copy of the waypoints, without status information. */
    WaypointSequence withoutStatus() const
    {
        WaypointSequence seq;
        for (const auto& w : waypoints) seq.waypoints.emplace_back(w);
        return seq;
    }

    /** Timestamp of user navigation command. */
    mrpt::system::TTimeStamp timestamp_nav_started = INVALID_TIMESTAMP;

    /** Whether the final waypoint has been reached successfuly. */
    bool final_goal_reached = false;

    /** Index in `waypoints` of the waypoint the navigator is currently trying
     * to reach.
     * This will point to the last waypoint after navigation ends successfully.
     * It has no value if navigation has not started yet */
    std::optional<waypoint_idx_t> waypoint_index_current_goal;

    /** Robot pose at last time step (has INVALID_NUM fields upon
     * initialization) */

    /** Ctor with default values */
    /** Gets navigation params as a human-readable format */
    std::string getAsText() const;

    /** Renders the sequence of waypoints (previous contents of `obj` are
     * cleared) */
    void getAsOpenglVisualization(
        mrpt::opengl::CSetOfObjects&    obj,
        const WaypointsRenderingParams& params = {}) const;
};
}  // namespace mpp
