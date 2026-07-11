/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/VehicleLocalizationState.h>
#include <mpp/data/VehicleOdometryState.h>
#include <mpp/follow/data/SampledTrajectory.h>
#include <mrpt/system/COutputLogger.h>

#include <chrono>
#include <cstdint>

namespace mpp
{
enum class StopKind : uint8_t
{
    REGULAR = 0,
    EMERGENCY
};

/** The ROS-free boundary between the TrajectoryFollower and a real platform.
 *
 * Unlike the older `VehicleMotionInterface` (PTG immediate/next command slots),
 * this interface is served a short **sampled predicted trajectory** to execute:
 * the follower has already closed the loop on localization, so the platform
 * side only needs a thin, fast inner servo that tracks the handed chunk
 * (feedforward `twist` + a small pose feedback) until the next one arrives.
 *
 * The kinematic model is not assumed here: it lives in the PTGs the follower
 * uses to shape commands, so this works for Ackermann / differential /
 * holonomic robots alike.
 */
class TrajectoryVehicleInterface : public mrpt::system::COutputLogger
{
   public:
    TrajectoryVehicleInterface()
        : mrpt::system::COutputLogger("TrajectoryVehicleInterface")
    {
    }
    virtual ~TrajectoryVehicleInterface() = default;

    /** Latest global localization (map frame). Leave `valid=false` on error. */
    virtual VehicleLocalizationState get_localization() = 0;

    /** Latest odometry (odom frame) + local velocity. Leave `valid=false` on
     * error. */
    virtual VehicleOdometryState get_odometry() = 0;

    /** Execute/track this short reference; it supersedes any previous one. An
     * empty trajectory means a controlled stop. Resets the watchdog. */
    virtual void follow(const SampledTrajectory& ref) = 0;

    /** Immediate stop request (regular or emergency). */
    virtual void stop(StopKind kind) = 0;

    /** Arm a watchdog: if neither follow() nor stop() is called within
     * `timeout`, the platform must stop the robot. */
    virtual void start_watchdog(std::chrono::milliseconds timeout) = 0;
};

}  // namespace mpp
