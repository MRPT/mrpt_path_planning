/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/ObstacleSource.h>

using namespace selfdrive;

ObstacleSource::~ObstacleSource() = default;

ObstacleSource::ObstacleSource(
    const mrpt::maps::CSimplePointsMap& staticObstacles)
{
    static_obs_ =
        std::make_shared<mrpt::maps::CSimplePointsMap>(staticObstacles);
}

mrpt::maps::CSimplePointsMap::Ptr ObstacleSource::obstacles([
    [maybe_unused]] mrpt::system::TTimeStamp t)
{
    MRPT_START

    if (static_obs_) return static_obs_;

    THROW_EXCEPTION(
        "Must either reimplement obstacles() or use ctor from static "
        "obstacles");

    MRPT_END
}
