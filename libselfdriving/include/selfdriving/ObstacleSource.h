/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/system/datetime.h>

namespace selfdriving
{
class ObstacleSource
{
   public:
    using Ptr = std::shared_ptr<ObstacleSource>;

    ObstacleSource() = default;
    virtual ~ObstacleSource();

    static Ptr FromStaticPointcloud(const mrpt::maps::CPointsMap::Ptr& pc);

    /** Returns all global obstacle points, in global "map" reference frame.
     */
    virtual mrpt::maps::CPointsMap::Ptr obstacles(
        mrpt::system::TTimeStamp t = mrpt::system::TTimeStamp()) = 0;

    virtual bool dynamic() const { return false; }
};

/** A simple obstacle source from a fixed (static world) point cloud. */
class ObstacleSourceStaticPointcloud : public ObstacleSource
{
   public:
    ObstacleSourceStaticPointcloud(
        const mrpt::maps::CPointsMap::Ptr& staticObstacles)
        : static_obs_(staticObstacles)
    {
        ASSERT_(static_obs_);
    }

    virtual mrpt::maps::CPointsMap::Ptr obstacles(
        [[maybe_unused]] mrpt::system::TTimeStamp t =
            mrpt::system::TTimeStamp()) override
    {
        return static_obs_;
    }

   private:
    mrpt::maps::CPointsMap::Ptr static_obs_;
};

}  // namespace selfdriving
