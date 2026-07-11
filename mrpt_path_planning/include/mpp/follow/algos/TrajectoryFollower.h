/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/VehicleLocalizationState.h>
#include <mpp/data/VehicleOdometryState.h>
#include <mpp/follow/data/SampledTrajectory.h>
#include <mpp/follow/data/Trajectory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/system/COutputLogger.h>

#include <cstdint>
#include <vector>

namespace mpp
{
enum class FollowerStatus : uint8_t
{
    /** No trajectory set, or fewer than 2 points. */
    Idle,
    /** Tracking normally. */
    Running,
    /** Final point reached within tolerance. */
    ReachedGoal,
    /** Stopped by the predictive safety layer beyond the block timeout (added
     * in the safety stage; never emitted by the pursuit core alone). */
    Blocked,
    /** Cross-track error exceeded the configured limit. */
    OffPathExceeded
};

/** Pure-pursuit trajectory follower core (ROS-free, no safety yet).
 *
 * Consumes a planner-agnostic pose+speed polyline (\ref Trajectory) and, each
 * control cycle, emits a short rolling \ref SampledTrajectory (the motion it
 * wants the robot to make over the next horizon) plus a status. Timing is
 * *spatial*: the polyline's speeds are caps along arc-length, never a schedule
 * to catch up to, so later safety slow-downs never cause rushing.
 *
 * Feedback: path progress is measured on the map-frame localization; the
 * emitted chunk is expressed in the odom frame (via the current map->odom
 * correction) so relocalization jumps do not lurch the wheels.
 *
 * This is the pursuit + speed-profile + events core. The predictive safety
 * layer (speed-scale/stop/resume) and the mvsim benchmark are added on top.
 */
class TrajectoryFollower : public mrpt::system::COutputLogger
{
   public:
    TrajectoryFollower();
    ~TrajectoryFollower() = default;

    struct Parameters
    {
        double max_speed         = 0.5;  //!< [m/s]
        double max_accel         = 0.5;  //!< [m/s^2]
        double max_decel         = 0.7;  //!< [m/s^2] (goal + speed reductions)
        double max_lateral_accel = 1.0;  //!< [m/s^2] curvature speed limit

        /** Lookahead distance L = clamp(lookahead_time * v, min, max) [m,s]. */
        double lookahead_min  = 0.4;
        double lookahead_max  = 1.5;
        double lookahead_time = 1.0;

        double goal_dist_tol   = 0.15;  //!< [m]
        double goal_ang_tol    = mrpt::DEG2RAD(12.0);  //!< [rad]
        double max_cross_track = 1.0;  //!< [m] OffPathExceeded

        double control_period = 0.05;  //!< [s] nominal call period (20 Hz)
        double horizon        = 1.5;  //!< [s] emitted chunk length
        double sample_period  = 0.1;  //!< [s] emitted chunk sampling

        std::string emit_frame = "odom";  //!< frame of the emitted chunk

        static Parameters      FromYAML(const mrpt::containers::yaml& c);
        mrpt::containers::yaml as_yaml() const;
        void                   load_from_yaml(const mrpt::containers::yaml& c);
    };

    Parameters params;

    /** Sets/replaces the reference path (hot-swappable at any time); resets
     * arc-length progress. Needs >= 2 points to track. */
    void setTrajectory(const Trajectory& traj);

    /** Clears the trajectory and progress. */
    void reset();

    bool   hasTrajectory() const { return traj_.size() >= 2; }
    double totalLength() const { return cumS_.empty() ? 0.0 : cumS_.back(); }

    struct Output
    {
        SampledTrajectory
            command;  //!< hand to TrajectoryVehicleInterface::follow()
        FollowerStatus status = FollowerStatus::Idle;

        // Debug / introspection:
        double               arc_length_s    = 0;
        double               cross_track_err = 0;  //!< signed [m]
        double               heading_err     = 0;  //!< [rad]
        double               target_speed    = 0;  //!< [m/s] immediate command
        mrpt::math::TPoint2D lookahead_point{0, 0};
    };

    /** One control cycle: given the latest localization (map) and odometry
     * (odom), computes and returns the command chunk + status. Does not itself
     * call the vehicle interface; the node does `iface.follow(out.command)` (or
     * `iface.stop()` on ReachedGoal). */
    Output step(
        const VehicleLocalizationState& loc, const VehicleOdometryState& odo);

   private:
    Trajectory          traj_;
    std::vector<double> cumS_;  //!< cumulative arc-length per point
    double              lastS_ = 0;  //!< monotonic progress (map projection)

    struct Projection
    {
        double s            = 0;  //!< arc-length of nearest point
        double cross_track  = 0;  //!< signed lateral error [m]
        double path_heading = 0;  //!< tangent heading [rad]
    };

    /** Nearest point on the polyline to `xy`, searching forward from `sHint`.
     * Pure (no state mutation). */
    Projection projectToPath(
        const mrpt::math::TPoint2D& xy, double sHint) const;

    /** Path point (x,y) at arc-length `s` (clamped to [0, total]). */
    mrpt::math::TPoint2D pointAtArc(double s) const;

    /** Interpolated speed cap at arc-length `s` (<=0 entries treated as
     * max_speed). */
    double speedCapAt(double s) const;

    struct Command
    {
        double               v     = 0;  //!< [m/s]
        double               omega = 0;  //!< [rad/s]
        mrpt::math::TPoint2D lookahead{0, 0};
    };

    /** Pure-pursuit command at `fromPose` given `currentV`, advancing the
     * lookahead from projection near `sHint`. `dt` bounds the accel/decel step.
     */
    Command pursuit(
        const mrpt::math::TPose2D& fromPose, double currentV, double sHint,
        double dt) const;
};

}  // namespace mpp
