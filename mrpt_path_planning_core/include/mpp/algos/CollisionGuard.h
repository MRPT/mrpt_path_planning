/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/TrajectoriesAndRobotShape.h>  // RobotShape
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/system/datetime.h>

#include <limits>
#include <optional>
#include <vector>

namespace mpp
{
/** Last-resort collision guard for velocity commands (ROS-free).
 *
 * Given the latest sensed obstacle points, expressed in the robot frame (so
 * this check does not depend on localization or on any reference path), it
 * limits a velocity command `(v, omega)` so that the robot can always brake to
 * a full stop before its footprint, inflated by `margin`, touches any of them.
 *
 * The robot is assumed to keep moving along the arc of constant curvature of
 * the command (or rotating in place, for `v = 0`) while it reacts and brakes.
 * The footprint is swept along that arc up to the stopping distance
 * `|v| * t_r + v^2 / (2 * a)`, where `t_r` is `reaction_time` plus the age of
 * the obstacle data, and `a` the braking deceleration. If a contact is found
 * at a free distance `d`, the speed is reduced to the largest one that can
 * still stop within `d`, keeping the curvature of the command.
 *
 * If the current robot velocity is given (e.g. from odometry), stopping
 * distances use the largest of the commanded and current speeds (a vehicle
 * still braking from a previous command is faster than the new one), and if
 * the current motion itself cannot stop before a contact, a full stop is
 * commanded.
 *
 * Fail-safe behaviors:
 * - No obstacle data, or older than `max_obstacles_age`: full stop.
 * - If the robot is already closer than `margin` to some obstacle (e.g. it
 *   was parked close to a wall), the margin is reduced to half that
 *   clearance, so it can always move along or away from it, but not closer.
 *   Points inside the footprint itself are ignored (and reported in
 *   Result::in_contact), since they are either sensor artifacts or an
 *   already-happened contact.
 *
 * Commands that stop the robot are never modified.
 */
class CollisionGuard
{
   public:
    CollisionGuard() = default;

    struct Parameters
    {
        /** [m/s^2] Braking deceleration the vehicle can surely achieve. */
        double max_decel = 1.0;

        /** [rad/s^2] Angular braking deceleration (in-place rotations, and
         * tight arcs). */
        double max_ang_decel = 2.0;

        /** [s] Worst-case time between deciding a command and the vehicle
         * starting to brake if needed: a command stays in effect for a whole
         * control period, plus the actuation latency of the platform. So it
         * must be at least the control period plus that latency. The age of
         * the obstacle data is added to it. */
        double reaction_time = 0.2;

        /** [m] Footprint inflation: minimum clearance to keep to obstacles. */
        double margin = 0.05;

        /** [s] Obstacle data older than this leads to a full stop. <= 0
         * disables the check. */
        double max_obstacles_age = 0.5;

        static Parameters      FromYAML(const mrpt::containers::yaml& c);
        mrpt::containers::yaml as_yaml() const;
        void                   load_from_yaml(const mrpt::containers::yaml& c);
    };

    Parameters params;

    /** Robot footprint (polygon or radius, in the robot frame). A
     * `std::monostate` shape means a point robot. */
    void setRobotShape(const RobotShape& shape);

    /** Sets the latest sensed obstacles, as (x,y) points in the robot frame,
     * and the time they were sensed. */
    void setObstacles(
        const std::vector<mrpt::math::TPoint2D>& pts,
        mrpt::system::TTimeStamp                 stamp);

    /** \overload (z coordinates are ignored: filter by height beforehand) */
    void setObstacles(
        const mrpt::maps::CPointsMap& pts, mrpt::system::TTimeStamp stamp);

    struct Result
    {
        double v     = 0;  //!< [m/s] safe linear speed
        double omega = 0;  //!< [rad/s] safe angular speed

        /** The command was reduced (or zeroed) by the guard. */
        bool limited = false;

        /** Missing or too old obstacle data: full stop. */
        bool stale = false;

        /** Some obstacle point is inside the (non-inflated) footprint. */
        bool in_contact = false;

        /** [m] or [rad] free travel along the command arc (or rotation)
         * before contact, within the checked stopping distance; +inf if
         * clear. */
        double free_travel = std::numeric_limits<double>::infinity();

        /** The obstacle point (robot frame) limiting the motion, if any. */
        std::optional<mrpt::math::TPoint2D> limiting_point;

        /** The robot was stopped because its current motion (rather than the
         * command) could not stop before a contact. */
        bool current_motion_unsafe = false;
    };

    /** Returns the safe version of the command `(v, omega)`, at time `now`
     * (used to evaluate the age of the obstacles). `currentVel` is the
     * current robot velocity (vx, omega are used), if known. */
    [[nodiscard]] Result filter(
        double v, double omega, mrpt::system::TTimeStamp now,
        const std::optional<mrpt::math::TTwist2D>& currentVel =
            std::nullopt) const;

    /** Free travel distance [m] along the arc of `(v, omega)` (in the
     * direction of `v`, which must be non-zero) before the inflated footprint
     * contacts an obstacle; +inf if none within `maxDist`. If `inContact` is
     * given, it is set to whether any point is inside the footprint. */
    double freeDistance(
        double v, double omega, double maxDist, bool* inContact = nullptr,
        std::optional<mrpt::math::TPoint2D>* contactPoint = nullptr) const;

    /** Free rotation [rad] in place (in the direction of `omega`) before the
     * inflated footprint contacts an obstacle; +inf if none within
     * `maxAngle`. */
    double freeAngle(
        double omega, double maxAngle, bool* inContact = nullptr,
        std::optional<mrpt::math::TPoint2D>* contactPoint = nullptr) const;

    /** Signed distance [m] from a robot-frame point to the footprint
     * (negative inside). */
    double signedDistance(const mrpt::math::TPoint2D& p) const;

   private:
    std::vector<mrpt::math::TPoint2D> poly_;  //!< empty: circle of radius_
    double                            radius_    = 0;
    double                            maxRadius_ = 0;  //!< circumradius

    std::vector<mrpt::math::TPoint2D> obstacles_;
    mrpt::system::TTimeStamp          obstaclesStamp_ = INVALID_TIMESTAMP;

    struct Motion
    {
        bool   rotation  = false;
        double curv      = 0;  //!< [1/m] (translations only)
        double a         = 0;  //!< braking deceleration
        double need      = 0;  //!< stopping travel
        double free      = 0;  //!< free travel
        bool   inContact = false;
        std::optional<mrpt::math::TPoint2D> contactPoint;
    };

    /** Free and needed stopping travel for the motion `(v, omega)`, with
     * `vStop` the speed (or angular speed, for rotations) to stop from. */
    Motion checkMotion(double v, double omega, double vStop, double tR) const;

    /** Sweeps the footprint along the poses given by `poseAt(t)` for
     * `t = 0, dt, 2*dt, ... <= maxT` and returns the last `t` before contact,
     * or +inf. */
    template <typename POSE_AT>
    double sweep(
        POSE_AT poseAt, double dt, double maxT, double reach, bool* inContact,
        std::optional<mrpt::math::TPoint2D>* contactPoint) const;
};

}  // namespace mpp
