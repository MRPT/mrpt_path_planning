/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/ObstacleSource.h>

using namespace selfdriving;

ObstacleSource::~ObstacleSource() = default;

ObstacleSource::Ptr ObstacleSource::FromStaticPointcloud(
    const mrpt::maps::CPointsMap::Ptr& pc)
{
    return std::make_shared<ObstacleSourceStaticPointcloud>(pc);
}
