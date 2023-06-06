/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/system/datetime.h>

namespace mpp
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

    mrpt::maps::CPointsMap::Ptr obstacles(
        [[maybe_unused]] mrpt::system::TTimeStamp t =
            mrpt::system::TTimeStamp()) override
    {
        return static_obs_;
    }

   private:
    mrpt::maps::CPointsMap::Ptr static_obs_;
};

/** Obstacles from a generic MRPT observation (2D lidar, 3D camera, velodyne,
 * etc.).
 * This creates a pointcloud with obstacles in the global nav frame, from the
 * raw observation data and a robot pose from an external localization system.
 */
class ObstacleSourceGenericSensor : public ObstacleSource
{
   public:
    ObstacleSourceGenericSensor() {}

    void set_sensor_observation(
        const mrpt::obs::CObservation::Ptr& o,
        const mrpt::poses::CPose3D&         robotPose)
    {
        auto lck         = mrpt::lockHelper(obsMtx_);
        obs_             = o;
        robotPoseForObs_ = robotPose;
    }

    mrpt::obs::CObservation::Ptr get_stored_sensor_observation() const
    {
        auto lck = mrpt::lockHelper(obsMtx_);
        return obs_;
    }

    mrpt::maps::CPointsMap::Ptr obstacles(
        [[maybe_unused]] mrpt::system::TTimeStamp t =
            mrpt::system::TTimeStamp()) override
    {
        auto lck = mrpt::lockHelper(obsMtx_);

        // TODO: Cached pts with obs timestamp for checking.
        auto pts = mrpt::maps::CSimplePointsMap::Create();
        if (obs_) { pts->insertObservation(*obs_, robotPoseForObs_); }

        return pts;
    }

   private:
    std::mutex                   obsMtx_;
    mrpt::obs::CObservation::Ptr obs_;
    mrpt::poses::CPose3D         robotPoseForObs_;
};

}  // namespace mpp
