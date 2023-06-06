/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/interfaces/ObstacleSource.h>

using namespace mpp;

ObstacleSource::~ObstacleSource() = default;

ObstacleSource::Ptr ObstacleSource::FromStaticPointcloud(
    const mrpt::maps::CPointsMap::Ptr& pc)
{
    return std::make_shared<ObstacleSourceStaticPointcloud>(pc);
}
