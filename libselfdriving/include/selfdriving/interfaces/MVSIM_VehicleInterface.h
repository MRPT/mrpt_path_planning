/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mvsim/Comms/Client.h>
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
        MRPT_LOG_INFO("Connecting to mvsim server...");
        connection_.connect();
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

   private:
    mvsim::Client connection_{"MVSIM_VehicleInterface"};
    std::string   robotName_ = "r1";
};

}  // namespace selfdriving
