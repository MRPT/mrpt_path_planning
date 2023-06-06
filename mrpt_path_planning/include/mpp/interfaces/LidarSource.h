/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/obs/CObservation2DRangeScan.h>

namespace mpp
{
class LidarSource
{
   public:
    /// Returns a copy of the last lidar observation
    virtual mrpt::obs::CObservation2DRangeScan::Ptr last_lidar_obs() const = 0;
};
}  // namespace mpp
