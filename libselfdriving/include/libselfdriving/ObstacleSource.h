/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <libselfdriving/MotionPrimitivesTree.h>
#include <libselfdriving/SE2_KinState.h>
#include <libselfdriving/ptg_t.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <limits>
#include <memory>
#include <vector>

namespace selfdrive
{
class ObstacleSource
{
   public:
    using Ptr = std::shared_ptr<ObstacleSource>;

    ObstacleSource() = default;
    ObstacleSource(const mrpt::maps::CSimplePointsMap& staticObstacles);
    virtual ~ObstacleSource();

    virtual mrpt::maps::CSimplePointsMap::Ptr obstacles(
        mrpt::system::TTimeStamp t = mrpt::system::TTimeStamp());

    virtual bool dynamic() const { return false; }

   private:
    mrpt::maps::CSimplePointsMap::Ptr static_obs_{};
};

}  // namespace selfdrive
