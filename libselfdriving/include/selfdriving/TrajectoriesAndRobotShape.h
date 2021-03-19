/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPolygon2D.h>
#include <selfdriving/ptg_t.h>

#include <memory>
#include <vector>

namespace selfdriving
{
struct RobotShape
{
    mrpt::math::TPolygon2D robot_shape;  //!< 2D robot shape
    double                 robot_radius{-1.0};  //!< Radius of circ. robot
};

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

}  // namespace selfdriving
