/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/GenericObservation.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>

namespace selfdriving
{
/** Vehicle adaptor class for the MVSIM simulator.
 *
 */
class MVSIM_VehicleInterface : public VehicleMotionInterface
{
   public:
    MVSIM_VehicleInterface() {}

    /** Connect to the MVSIM server.
     */
    void connect()
    {
        connection_.enable_profiler(true);
        MRPT_LOG_INFO("Connecting to mvsim server...");
        connection_.connect();

        connection_.subscribeTopic<mvsim_msgs::GenericObservation>(
            mrpt::format("/%s/%s", robotName_.c_str(), lidarName_.c_str()),
            [this](const mvsim_msgs::GenericObservation& o) { onLidar(o); });

        MRPT_LOG_INFO("Connected OK.");
    }

    // See base class docs
    VehicleLocalizationState get_localization() override
    {
        mvsim_msgs::SrvGetPose req;
        req.set_objectid(robotName_);

        mvsim_msgs::SrvGetPoseAnswer ans;
        connection_.callService("get_pose", req, ans);

        VehicleLocalizationState vls;
        vls.frame_id  = "map";
        vls.timestamp = mrpt::Clock::now();
        vls.valid     = true;

        vls.pose.x   = ans.pose().x();
        vls.pose.y   = ans.pose().y();
        vls.pose.phi = ans.pose().yaw();

        return vls;
    }

    // See base class docs
    VehicleOdometryState get_odometry() override
    {
        //
        return {};
    }

    // See base class docs
    bool motion_execute(
        const std::optional<CVehicleVelCmd::Ptr>& immediate,
        const std::optional<EnqueuedMotionCmd>&   next) override
    {
        return true;
    }

    // See base class docs
    void stop(const STOP_TYPE stopType) override
    {
        //
    }

    /// Returns a copy of the last lidar observation
    mrpt::obs::CObservation2DRangeScan::Ptr last_lidar_obs() const
    {
        auto lck = mrpt::lockHelper(lastLidarObsMtx_);
        return lastLidarObs_;
    }

   private:
    mvsim::Client connection_{"MVSIM_VehicleInterface"};
    std::string   robotName_ = "r1";
    std::string   lidarName_ = "laser1";

    std::mutex                              lastLidarObsMtx_;
    mrpt::obs::CObservation2DRangeScan::Ptr lastLidarObs_;

    void onLidar(const mvsim_msgs::GenericObservation& o)
    {
        try
        {
            const std::vector<uint8_t> data(
                o.mrptserializedobservation().begin(),
                o.mrptserializedobservation().end());

            mrpt::serialization::CSerializable::Ptr obj;
            mrpt::serialization::OctetVectorToObject(data, obj);

            auto lck = mrpt::lockHelper(lastLidarObsMtx_);
            lastLidarObs_ =
                std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(
                    obj);
            ASSERT_(lastLidarObs_);

            // MRPT_LOG_DEBUG_STREAM("sensor callback: " <<
            // lastLidarObs_->getDescriptionAsTextValue());
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_ERROR_STREAM(e.what());
        }
    }
};

}  // namespace selfdriving
