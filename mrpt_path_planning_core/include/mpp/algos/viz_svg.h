/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/PlannerOutput.h>

#include <string>

namespace mpp
{
/** Options for the 2D SVG plan exporter (save_plan_to_svg / plan_to_svg).
 *  Colors are SVG color strings (e.g. "#cc0000" or "none"). */
struct SvgExportOptions
{
    double image_width_px =
        900;  //!< target image width; height auto from aspect
    double margin_px = 25;

    bool draw_obstacles    = true;
    bool draw_tree         = true;  //!< full motion tree (faint)
    bool draw_path         = true;  //!< solution / best path (bold)
    bool draw_robot_shapes = true;  //!< footprint along the path
    bool draw_start_goal   = true;
    bool draw_bbox         = true;
    bool draw_scalebar     = true;

    /** Draw one robot footprint out of every N interpolated path poses
     *  (plus always at start and goal). 0 => only at start and goal. */
    size_t robot_shape_decimation = 8;

    /** Plot one obstacle point out of every N (1 = all). */
    size_t obstacle_decimation = 1;

    /** Draw one motion-tree edge out of every N (1 = all). Use a larger value
     *  to thin out a very dense exhaustive-search tree (e.g. failed queries)
     *  into a visually usable figure instead of tens of thousands of edges. */
    size_t tree_decimation = 1;

    std::string color_background = "#ffffff";
    std::string color_obstacles  = "#2255cc";
    std::string color_tree       = "#bdbdbd";
    std::string color_path       = "#cc0000";
    std::string color_robot      = "#cc0000";
    std::string color_robot_fill = "none";
    std::string color_start      = "#118811";
    std::string color_goal       = "#cc0000";
    std::string color_bbox       = "#888888";

    double stroke_path_px     = 2.5;
    double stroke_tree_px     = 0.6;
    double stroke_robot_px    = 1.0;
    double obstacle_radius_px = 1.4;
};

/** Returns an SVG document (as a string) with a top-down 2D plot of a planner
 *  result: world bounding box, obstacles, the motion tree, the solution (or
 *  best) path, the robot footprint along it, and start/goal markers. Intended
 *  for debugging and for vector figures in papers (no OpenGL/GUI needed). */
std::string plan_to_svg(
    const PlannerOutput& plan, const SvgExportOptions& opts = {});

/** Writes plan_to_svg() to `filename`. Returns false on I/O error. */
bool save_plan_to_svg(
    const PlannerOutput& plan, const std::string& filename,
    const SvgExportOptions& opts = {});

}  // namespace mpp
