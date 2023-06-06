/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
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

    void initFromConfigFile(
        mrpt::config::CConfigFileBase& cfg, const std::string& section);
    // void initFromYAML(const mrpt::containers::yaml& node);

    std::vector<std::shared_ptr<ptg_t>> ptgs;  //!< Allowed movement sets
    RobotShape                          robotShape;

   private:
    bool initialized_ = false;
};

bool obstaclePointCollides(
    const mrpt::math::TPoint2D&      obstacleWrtRobot,
    const TrajectoriesAndRobotShape& trs);

}  // namespace mpp
