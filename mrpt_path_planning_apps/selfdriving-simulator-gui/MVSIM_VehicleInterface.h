/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/interfaces/LidarSource.h>
#include <mpp/interfaces/TrajectoryVehicleInterface.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwist.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwistAnswer.pb.h>

#include <mutex>
#include <string>

namespace mpp
{
/** Vehicle adaptor for the MVSIM simulator, for the TrajectoryFollower.
 *
 * Commands are sent as twist setpoints of the vehicle controller, and both
 * localization and odometry are the simulator ground truth.
 */
class MVSIM_VehicleInterface : public TrajectoryVehicleInterface,
                               public LidarSource
{
   public:
    /** \param robotName The name of the vehicle in the MVSIM world. */
    explicit MVSIM_VehicleInterface(const std::string& robotName)
        : robotName_(robotName)
    {
    }

    /** Connects to the MVSIM server. */
    void connect()
    {
        MRPT_LOG_INFO("Connecting to mvsim server...");
        connection_.connect();
        MRPT_LOG_INFO("Connected OK.");
    }

    const std::string& robot_name() const { return robotName_; }

    /** To be fed with the observations of this robot. 2D scans from the first
     * 2D lidar are stored for last_lidar_obs(), others are ignored. */
    void on_observation(const mrpt::obs::CObservation::Ptr& obs)
    {
        auto scan =
            std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(obs);
        if (!scan) { return; }
        auto lck = mrpt::lockHelper(lastLidarObsMtx_);
        if (lidarLabel_.empty())
        {
            lidarLabel_ = scan->sensorLabel;
            MRPT_LOG_INFO_STREAM("Using lidar: " << lidarLabel_);
        }
        if (scan->sensorLabel == lidarLabel_) { lastLidarObs_ = scan; }
    }

    VehicleLocalizationState get_localization() override
    {
        const auto ans = get_pose();

        VehicleLocalizationState vls;
        vls.frame_id  = "map";
        vls.timestamp = mrpt::Clock::now();
        vls.valid     = ans.success();
        vls.pose.x    = ans.pose().x();
        vls.pose.y    = ans.pose().y();
        vls.pose.phi  = ans.pose().yaw();
        return vls;
    }

    VehicleOdometryState get_odometry() override
    {
        const auto ans = get_pose();

        // Ground truth: the odom frame is the map frame.
        VehicleOdometryState vos;
        vos.odometry.x   = ans.pose().x();
        vos.odometry.y   = ans.pose().y();
        vos.odometry.phi = ans.pose().yaw();

        // The answer twist is in the vehicle frame:
        vos.odometryVelocityLocal = mrpt::math::TTwist2D(
            ans.twist().vx(), ans.twist().vy(), ans.twist().wz());

        vos.timestamp = mrpt::Clock::now();
        vos.valid     = ans.success();
        return vos;
    }

    void follow(const SampledTrajectory& ref) override
    {
        mrpt::math::TTwist2D tw;
        if (!ref.empty()) { tw = ref.points.front().twist; }
        set_twist(tw);
    }

    void stop([[maybe_unused]] StopKind kind) override
    {
        set_twist(mrpt::math::TTwist2D(0, 0, 0));
    }

    // The control loop of this app sends commands at a fixed rate:
    void start_watchdog(
        [[maybe_unused]] std::chrono::milliseconds timeout) override
    {
    }

    mrpt::obs::CObservation2DRangeScan::Ptr last_lidar_obs() const override
    {
        auto lck = mrpt::lockHelper(lastLidarObsMtx_);
        return lastLidarObs_;
    }

   private:
    mvsim::Client connection_{"MVSIM_VehicleInterface"};
    std::string   robotName_;
    std::string   lidarLabel_;

    mutable std::mutex                      lastLidarObsMtx_;
    mrpt::obs::CObservation2DRangeScan::Ptr lastLidarObs_;

    mvsim_msgs::SrvGetPoseAnswer get_pose()
    {
        mvsim_msgs::SrvGetPose req;
        req.set_objectid(robotName_);
        mvsim_msgs::SrvGetPoseAnswer ans;
        connection_.callService("get_pose", req, ans);
        return ans;
    }

    void set_twist(const mrpt::math::TTwist2D& t)
    {
        mvsim_msgs::SrvSetControllerTwist req;
        req.set_objectid(robotName_);
        auto* tw = req.mutable_twistsetpoint();
        tw->set_vx(t.vx);
        tw->set_vy(t.vy);
        tw->set_vz(0);
        tw->set_wx(0);
        tw->set_wy(0);
        tw->set_wz(t.omega);

        mvsim_msgs::SrvSetControllerTwistAnswer ans;
        connection_.callService("set_controller_twist", req, ans);
        if (!ans.success())
        {
            MRPT_LOG_THROTTLE_ERROR(
                5.0, "set_controller_twist() failed for the vehicle.");
        }
    }
};

}  // namespace mpp
