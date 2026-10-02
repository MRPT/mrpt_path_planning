/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/ptg_t.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPolygon2D.h>

#include <memory>
#include <variant>
#include <vector>

namespace mpp
{
using robot_radius_t = double;

using RobotShape =
    std::variant<mrpt::math::TPolygon2D, robot_radius_t, std::monostate>;

class TrajectoriesAndRobotShape
{
   public:
    TrajectoriesAndRobotShape()  = default;
    ~TrajectoriesAndRobotShape() = default;

    bool initialized() const { return initialized_; }
    void clear();

    /** Loads the robot description (shape, kinematics) and the PTGs from a
     * config file section.
     *
     * If `initializePTGs` is true (default), PTGs are initialized, which
     * builds (or loads from a cache file) their collision grids, as needed by
     * planners. Users only needing the robot description (e.g. a path
     * follower) can skip it, which is much faster.
     *
     * Collision grids are cached in `$XDG_CACHE_HOME/mrpt_path_planning` (or
     * `~/.cache/mrpt_path_planning`), with file names derived from the PTG and
     * robot shape parameters, so different configurations do not invalidate
     * each other's cache. */
    void initFromConfigFile(
        mrpt::config::CConfigFileBase& cfg, const std::string& section,
        bool initializePTGs = true);
    // void initFromYAML(const mrpt::containers::yaml& node);

    std::vector<std::shared_ptr<ptg_t>> ptgs;  //!< Allowed movement sets
    RobotShape                          robotShape;

    /** [m] Tightest turning radius the vehicle can physically make (e.g.
     * steering-limited, for Ackermann vehicles); 0 if it can turn in place.
     * Loaded from `RobotModel_min_turning_radius` (default: 0). It is part of
     * the robot description so that every consumer (planner, path follower)
     * shares one value, and PTGs are checked against it when loaded: a
     * DiffDrive_C PTG turning tighter than this is a configuration error. */
    double minTurningRadius = 0;

   private:
    bool initialized_ = false;
};

bool obstaclePointCollides(
    const mrpt::math::TPoint2D&      obstacleWrtRobot,
    const TrajectoriesAndRobotShape& trs);

}  // namespace mpp
