/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/SampledTrajectory.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>  // RobotShape
#include <mpp/data/Trajectory.h>
#include <mpp/data/VehicleLocalizationState.h>
#include <mpp/data/VehicleOdometryState.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/system/COutputLogger.h>

#include <cstdint>
#include <limits>
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

/** Pure-pursuit trajectory follower core with predictive safety (ROS-free).
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
 * Predictive safety: if a robot footprint and live obstacle points are set
 * (\ref setRobotShape / \ref setObstacles), each cycle the follower sweeps the
 * footprint over both its own forecast rollout and the upcoming reference
 * segment, and scales the commanded speed down (to a full stop before contact),
 * resuming automatically when the way clears. With no footprint or no obstacles
 * this layer is inert and the follower behaves as a plain pursuit core.
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

        /** [m] If > 0, the tightest turn radius the vehicle can physically
         * make (e.g. an Ackermann robot's steering-limited minimum radius).
         * The pure-pursuit curvature is clamped to 1/min_turn_radius before
         * being used, so the follower never *commands* a turn tighter than
         * the vehicle can actually execute -- unlike max_lateral_accel alone,
         * which only trades off speed for curvature but never bounds
         * curvature itself, so it can still ask for an arbitrarily tight
         * (just slow) turn. <= 0 disables the clamp (differential-drive /
         * unicycle, any curvature achievable). */
        double min_turn_radius = 0.0;

        /** Curvature-adaptive lookahead: the lookahead point is marched forward
         * along the path from the projection to where the path has bent
         * (accumulated |turned angle|) by `lookahead_bend`, capped at
         * `lookahead_max` travel and never past the next cusp. Geometry-driven,
         * so it needs no per-speed schedule to tune: on a straight run the bend
         * never accumulates and the full cap distance is used (smooth
         * tracking); through a curve or into the terminal maneuver the
         * lookahead shortens automatically to trace it tightly. */
        double lookahead_max  = 1.5;  //!< [m] max lookahead travel
        double lookahead_bend = mrpt::DEG2RAD(25.0);  //!< [rad] path bend that
                                                      //!< caps the lookahead

        double goal_dist_tol   = 0.15;  //!< [m]
        double goal_ang_tol    = mrpt::DEG2RAD(12.0);  //!< [rad]
        double max_cross_track = 1.0;  //!< [m] OffPathExceeded

        /** [m] Once the robot settles within this distance of the goal and has
         * passed its closest approach, it latches "arrived" and holds a stop,
         * so it never drives back away from a goal it cannot perfectly seat (no
         * runaway / thrashing on a kinematically infeasible final pose). */
        double arrival_radius = 0.3;

        double control_period = 0.05;  //!< [s] nominal call period (20 Hz)
        double horizon        = 1.5;  //!< [s] emitted chunk length
        double sample_period  = 0.1;  //!< [s] emitted chunk sampling

        std::string emit_frame = "odom";  //!< frame of the emitted chunk

        // --- Predictive safety (inert unless a footprint + obstacles are set)
        // ---

        /** [m] Footprint clearance at or below which a swept pose counts as a
         * predicted contact (a small inflation of the footprint). */
        double safety_margin = 0.05;

        /** [m] If the nearest predicted contact is within this travel distance,
         * command a full stop (speed scale 0). */
        double stop_distance = 0.3;

        /** [m] If the nearest predicted contact is beyond this travel distance,
         * do not slow down at all (speed scale 1). Linear scaling in between.
         */
        double slow_distance = 1.5;

        /** [s] How far ahead to roll out the command forecast for the safety
         * sweep. */
        double safety_horizon = 3.0;

        /** [m] How far ahead along the reference path to sweep the footprint.
         */
        double reference_lookahead_dist = 2.5;

        /** [m] Step used to sample the footprint boundary and to march the
         * reference sweep. */
        double footprint_sample_resolution = 0.1;

        /** Speed scale that must be recovered before resuming after a safety
         * stop (hysteresis to avoid chattering). */
        double resume_scale = 0.2;

        /** [s] If held stopped by the safety layer longer than this, report
         * `Blocked`. */
        double block_timeout = 5.0;

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

    /** Sets the robot footprint used by the predictive safety sweeps (polygon,
     * radius, or `std::monostate` to sample only the reference point). */
    void setRobotShape(const RobotShape& shape);

    /** Sets/replaces the live obstacle points (map frame) used by the
     * predictive safety sweeps. An empty cloud disables safety scaling. */
    void setObstacles(const mrpt::maps::CPointsMap& obstacles);

    /** Convenience overload: obstacle points as (x,y) in the map frame. */
    void setObstacles(const std::vector<mrpt::math::TPoint2D>& obstacles);

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

        /** Safety speed scale applied this cycle (1 = unrestricted, 0 = stopped
         * by the predictive safety layer). */
        double safety_scale = 1.0;
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
    std::vector<double>
        cuspS_;  //!< arc-lengths where travel direction reverses
    std::vector<std::size_t>
           cuspIdx_;  //!< traj_ knot index for each entry in cuspS_
    double lastS_ = 0;  //!< monotonic progress (map projection)

    /** Driving gear (+1 forward, -1 reverse) for each cusp-bounded interval
     * of the path (size == cuspS_.size() + 1); see \ref advanceGear. */
    std::vector<double> gearPerInterval_;

    /** Index into gearPerInterval_ of the interval the follower has last
     * committed to; monotonic (see \ref advanceGear). */
    std::size_t currentInterval_ = 0;

    /** Last commanded speed [m/s]. The feedforward speed ramp is rate-limited
     * from this internal state (not from measured odometry velocity), so the
     * profile still accelerates when the odometry source reports no twist. */
    double lastCommandedSpeed_ = 0;

    /** Latched once the robot has settled within `arrival_radius` of the goal;
     * from then on it holds a stop instead of driving back away from a pose it
     * cannot perfectly seat. */
    bool arrived_ = false;

    /** Closest distance to the goal point reached so far, used to detect when
     * the robot has passed its closest approach and would start driving away.
     */
    double minDistToGoal_ = std::numeric_limits<double>::infinity();

    // Predictive safety state:
    std::vector<mrpt::math::TPoint2D>
                                 shapeSamples_;  //!< footprint, robot frame
    mrpt::maps::CSimplePointsMap obstacles_;  //!< map frame (kd-tree)
    bool stopped_ = false;  //!< safety-stop hysteresis latch
    mrpt::system::TTimeStamp stoppedSince_ = INVALID_TIMESTAMP;

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

    /** Curvature-adaptive lookahead arc-length: marches forward from `sStart`
     * accumulating the path's |turned angle| and returns the arc-length where
     * it first reaches `params.lookahead_bend`, clamped to `[sStart + floor,
     * capS]`. A small floor keeps the lookahead off the robot itself (an
     * arbitrarily short lookahead blows up the pure-pursuit curvature). */
    double adaptiveLookaheadS(double sStart, double capS) const;

    /** Advances \ref currentInterval_ past every cusp whose arc-length the
     * localized projection `s` has already reached, and returns that
     * interval's gear (\ref gearPerInterval_). Monotonic and latched (never
     * regresses), called once per real \ref step cycle -- not re-evaluated
     * per predicted sample -- so a projection that lingers at/near a cusp
     * (e.g. while nearly stopped there, or pinned on the shared vertex of a
     * path that loops back close to itself, see \ref projectToPath) commits
     * to the new gear instead of chattering back and forth. A path with a
     * genuine mid-path direction reversal (e.g. a three-point turn) needs a
     * different gear before and after the cusp; using a single gear for the
     * whole path would make the follower cut straight across such a
     * maneuver instead of tracing it. */
    double advanceGear(double s);

    struct Command
    {
        double               v     = 0;  //!< [m/s]
        double               omega = 0;  //!< [rad/s]
        mrpt::math::TPoint2D lookahead{0, 0};
    };

    /** Pure-pursuit command at `fromPose` given `currentV`, advancing the
     * lookahead from projection near `sHint`. `dt` bounds the accel/decel step.
     * `speedScale` (0..1) caps the target speed before rate-limiting (safety).
     * `gear` (+1 forward, -1 reverse) sets the travel direction; the caller
     * decides it once per cycle (with hysteresis) and holds it over the
     * horizon.
     */
    Command pursuit(
        const mrpt::math::TPose2D& fromPose, double currentV, double sHint,
        double dt, double gear, double speedScale = 1.0) const;

    /** Pose (x,y + tangent heading) on the reference polyline at arc-length
     * `s`. */
    mrpt::math::TPose2D poseAtArc(double s) const;

    /** Min distance from the footprint (at map-frame pose `p`) to the nearest
     * obstacle point; +inf if no obstacles are set. */
    double footprintClearance(const mrpt::math::TPose2D& p) const;

    /** Rolls the command forecast forward from `startPose` (map frame) and
     * returns the travel distance to the first predicted footprint contact;
     * +inf if none within `safety_horizon`. `gear` is the interval gear
     * already latched by \ref advanceGear for this cycle (the forecast does
     * not re-decide it per predicted sample). */
    double forecastContactDistance(
        const mrpt::math::TPose2D& startPose, double startV, double startS,
        double gear) const;

    /** Sweeps the footprint along the reference path ahead of `startS` and
     * returns the travel distance to the first predicted contact; +inf if none
     * within `reference_lookahead_dist`. */
    double referenceContactDistance(double startS) const;

    /** Maps a nearest-contact travel distance to a [0,1] speed scale. */
    double contactDistanceToScale(double d) const;
};

}  // namespace mpp
