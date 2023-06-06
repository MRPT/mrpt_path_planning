/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/interfaces/LidarSource.h>
#include <mpp/interfaces/VehicleMotionInterface.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/GenericObservation.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwist.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwistAnswer.pb.h>

namespace mpp
{
/** Vehicle adaptor class for the MVSIM simulator.
 *
 * Motion commands implemented here in motion_execute():
 *  - mrpt::nav::CVehicleSimul_DiffDriven: For ackermann-like steering.
 *  - mrpt::nav::CVehicleSimul_Holo: For holonomic-like steering.
 *
 * \note This file must be implemented in the .h to avoid a direct dependency
 *       of this library on mvsim headers. Only if the user project uses this,
 *       it must then depend on mvsim.
 */
class MVSIM_VehicleInterface : public VehicleMotionInterface, public LidarSource
{
    DEFINE_MRPT_OBJECT(MVSIM_VehicleInterface, mpp)

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
        mvsim_msgs::SrvGetPose req;
        req.set_objectid(robotName_);

        mvsim_msgs::SrvGetPoseAnswer ans;
        connection_.callService("get_pose", req, ans);

        VehicleOdometryState vos;
        vos.odometry.x   = ans.pose().x();
        vos.odometry.y   = ans.pose().y();
        vos.odometry.phi = ans.pose().yaw();

        vos.odometryVelocityLocal.vx    = ans.twist().vx();
        vos.odometryVelocityLocal.vy    = ans.twist().vy();
        vos.odometryVelocityLocal.omega = ans.twist().wz();

        vos.pendedActionExists = false;  // TODO!
        vos.timestamp          = mrpt::Clock::now();
        vos.valid              = true;

        return vos;
    }

    // See base class docs
    bool motion_execute(
        const std::optional<CVehicleVelCmd::Ptr>& immediate,
        const std::optional<EnqueuedMotionCmd>&   next) override
    {
        if (next.has_value())
        {
            THROW_EXCEPTION(
                "Enqueued actions not implemented yet in this wrapper.");
        }
        if (!immediate.has_value())
        {
            // a NOP:
            // TODO
            return true;  // ok
        }

        // A regular immediate cmd:
        // We map this request into a service call "set_controller_twist()".
        mvsim_msgs::SrvSetControllerTwist req;
        req.set_objectid(robotName_);
        auto* tw = req.mutable_twistsetpoint();
        tw->set_vz(0);
        tw->set_wx(0);
        tw->set_wy(0);

        if (auto cmdDiff = std::dynamic_pointer_cast<
                mrpt::kinematics::CVehicleVelCmd_DiffDriven>(immediate.value());
            cmdDiff)
        {
            tw->set_vx(cmdDiff->lin_vel);
            tw->set_vy(0);
            tw->set_wz(cmdDiff->ang_vel);
        }
        else if (auto cmdHolo = std::dynamic_pointer_cast<
                     mrpt::kinematics::CVehicleVelCmd_Holo>(immediate.value());
                 cmdHolo)
        {
            tw->set_vx(cmdHolo->vel * std::cos(cmdHolo->dir_local));
            tw->set_vy(cmdHolo->vel * std::sin(cmdHolo->dir_local));
            tw->set_wz(cmdHolo->rot_speed);
        }
        else
        {
            MRPT_LOG_ERROR("Unhandled class received in motion_execute().");
            return false;
        }

        mvsim_msgs::SrvSetControllerTwistAnswer ans;
        connection_.callService("set_controller_twist", req, ans);

        return ans.success();
    }

    // See base class docs
    void stop([[maybe_unused]] const STOP_TYPE stopType) override
    {
        //
    }

    /// Returns a copy of the last lidar observation
    mrpt::obs::CObservation2DRangeScan::Ptr last_lidar_obs() const override
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

}  // namespace mpp

IMPLEMENTS_MRPT_OBJECT(MVSIM_VehicleInterface, VehicleMotionInterface, mpp)
