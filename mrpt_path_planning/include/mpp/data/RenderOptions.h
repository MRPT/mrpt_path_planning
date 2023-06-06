/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/graphs/TNodeID.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>

#include <optional>
#include <string>

namespace mpp
{
/** Options for render_tree()  */
struct RenderOptions
{
    RenderOptions()  = default;
    ~RenderOptions() = default;

    /// If set, highlight the path from root towards this node (e.g. the target)
    std::optional<mrpt::graphs::TNodeID> highlight_path_to_node_id;

    bool highlight_last_added_edge = false;

    /// Draw one out of N vehicle shapes along the highlighted
    size_t draw_shape_decimation = 1;

    std::optional<mrpt::math::TPose2D> x_rand_pose;
    std::optional<mrpt::math::TPose2D> new_state;

    std::optional<mrpt::math::TPose2D>           x_nearest_pose;
    std::optional<const mrpt::maps::CPointsMap*> local_obs_from_nearest_pose;

    /// A scale factor to all XYZ corners (default=auto from vehicle shape)
    std::optional<double> xyzcorners_scale;

    /// 0:disable, not assigned: auto
    std::optional<double> ground_xy_grid_frequency;

    /// If !=0, represent different headings as different heights:
    double phi2z_scale = 1.0;

    // RGB + A colors:
    mrpt::img::TColor color_vehicle{0xff0000, 0xff};
    mrpt::img::TColor color_obstacles{0x0000ff, 0x40};
    mrpt::img::TColor color_local_obstacles{0x0000ff, 0xff};
    mrpt::img::TColor color_start{0x000000, 0xff};
    mrpt::img::TColor color_goal{0x000000, 0xff};
    mrpt::img::TColor color_ground_xy_grid{0xffffff, 0xff};
    mrpt::img::TColor color_normal_edge{0x222222, 0x40};
    mrpt::img::TColor color_last_edge{0xffff00, 0xff};
    mrpt::img::TColor color_optimal_edge{0x000000, 0xff};

    float width_last_edge            = 3.f;  //!< 0=hidden
    float width_normal_edge          = 1.f;  //!< 0=hidden
    float width_optimal_edge         = 4.f;  //!< 0=hidden
    float point_size_obstacles       = 5.f;
    float point_size_local_obstacles = 5.f;

    /** (Default=0.01) Height (Z coordinate) for the vehicle shapes. Helps
     * making it in the "first plane" */
    double vehicle_shape_z = 0.01;

    /** Robot line width for visualization - default 2.0 */
    double vehicle_line_width = 2.0;

    bool draw_obstacles = true;

    bool   draw_twist        = true;
    double linVelScale       = 0.25;
    double angVelScale       = 0.25;
    double twistArrowsRadius = 0.01;

    bool   showEdgeCosts     = false;
    double edgeCostLabelSize = 0.025;

    std::string          log_msg;
    mrpt::math::TPoint3D log_msg_position;
    double               log_msg_scale = 0.2;
};

}  // namespace mpp
