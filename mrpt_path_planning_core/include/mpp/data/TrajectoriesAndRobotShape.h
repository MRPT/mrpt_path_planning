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

#include <functional>
#include <memory>
#include <string>
#include <variant>
#include <vector>

/** Feature macro: TrajectoriesAndRobotShape::clone() exists and PTG copies are
 * independent, so consumers can `#if defined(MPP_HAS_PTG_CLONE)` to stay
 * compatible with older versions. */
#define MPP_HAS_PTG_CLONE 1

/** Feature macro: TrajectoriesAndRobotShape::initFromConfigFile() accepts an
 * InitProgressCallback. */
#define MPP_HAS_PTG_INIT_PROGRESS 1

namespace mpp
{
using robot_radius_t = double;

using RobotShape =
    std::variant<mrpt::math::TPolygon2D, robot_radius_t, std::monostate>;

/** Called by TrajectoriesAndRobotShape::initFromConfigFile() for each PTG, as
 * soon as it is known whether its collision grid is loaded from the cache
 * (`fromCache` true: fast) or built (false: slow, it starts right after the
 * call, and it is also false for PTGs without a grid): the PTG index, the
 * total number of PTGs and the PTG description. A cache file that exists but
 * cannot be used (e.g. made for another configuration or corrupt) counts as a
 * build. It is called once more at the end, with `index == count`, so callers
 * know the initialization has finished. */
using InitProgressCallback = std::function<void(
    size_t index, size_t count, const std::string& ptgDescription,
    bool fromCache)>;

class TrajectoriesAndRobotShape
{
   public:
    TrajectoriesAndRobotShape()  = default;
    ~TrajectoriesAndRobotShape() = default;

    bool initialized() const { return initialized_; }
    void clear();

    /** A deep copy, with its own PTG instances. Evaluating a PTG changes its
     * internal state (dynamic state, caches), so concurrent users (e.g.
     * planners running in different threads) must each use their own copy.
     * Copying this object instead shares the PTG instances. */
    [[nodiscard]] TrajectoriesAndRobotShape clone() const;

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
     * each other's cache.
     *
     * If given, `progress` is called while initializing the PTGs (so only if
     * `initializePTGs` is true), from the calling thread. */
    void initFromConfigFile(
        mrpt::config::CConfigFileBase& cfg, const std::string& section,
        bool initializePTGs = true, const InitProgressCallback& progress = {});
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
