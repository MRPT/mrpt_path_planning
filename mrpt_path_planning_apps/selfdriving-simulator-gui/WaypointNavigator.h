/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/TrajectoryFollower.h>
#include <mpp/data/PlannerOutput.h>
#include <mpp/data/Trajectory.h>
#include <mpp/data/Waypoints.h>
#include <mpp/interfaces/LidarSource.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mpp/interfaces/TrajectoryVehicleInterface.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace mpp
{
/** Waypoint navigation for the simulator app: each leg between waypoints is
 * planned with TPS_Astar, the legs are joined into one reference path, and the
 * TrajectoryFollower drives the vehicle along it.
 *
 * Planning runs in a background thread, and the follower in its own control
 * loop thread, so all public methods return immediately.
 */
class WaypointNavigator : public mrpt::system::COutputLogger
{
   public:
    WaypointNavigator();
    ~WaypointNavigator();

    WaypointNavigator(const WaypointNavigator&)            = delete;
    WaypointNavigator& operator=(const WaypointNavigator&) = delete;

    struct Config
    {
        TrajectoriesAndRobotShape ptgs;
        TPS_Astar_Parameters      plannerParams;

        /// Cost map used to keep paths away from static obstacles:
        std::optional<CostEvaluatorCostMap::Parameters> globalCostParams;

        /// [m] Margin around start and goal for the planning area.
        double plannerBboxMargin = 4.0;

        std::shared_ptr<TrajectoryVehicleInterface> vehicle;
        /// Optional: live obstacles for the follower predictive safety.
        std::shared_ptr<LidarSource> lidar;

        /// Where to draw the plan and the follower state, and functions to
        /// lock/unlock it while modifying it:
        mrpt::viz::CSetOfObjects::Ptr vizScene;
        std::function<void()>         on_viz_pre_modify;
        std::function<void()>         on_viz_post_modify;
    };

    Config             config;
    TrajectoryFollower follower;

    /** Sets the static obstacles used for planning (map frame). */
    void set_static_obstacles(const mrpt::maps::CPointsMap::Ptr& pts);

    /** Starts the control loop. Call after filling in `config`. */
    void start();

    /** Plans and follows a path through the given waypoints, starting at the
     * current vehicle pose. Ignored while another request is being planned. */
    void request_navigation(const WaypointSequence& wps);

    /** Plans a single path, without driving the vehicle. The result is drawn
     * with the full search tree, and can be followed with follow_last_plan().
     * Ignored while another request is being planned. */
    void request_single_plan(
        const SE2_KinState& start, const SE2orR2_KinState& goal);

    /** Follows the last successful plan from request_single_plan(). */
    void follow_last_plan();

    void suspend();
    void resume();
    void cancel();

    /** Human-readable status, e.g. for a GUI label. */
    std::string status_text() const;

   private:
    std::thread        controlThread_;
    std::thread        plannerThread_;
    std::atomic_bool   closing_{false};
    std::atomic_bool   planning_{false};
    std::atomic_bool   suspended_{false};
    mutable std::mutex mtx_;  //!< follower, status, staticObstacles_

    mrpt::maps::CPointsMap::Ptr  staticObstacles_;
    std::string                  status_        = "Idle";
    bool                         driving_       = false;
    mrpt::system::TTimeStamp     lastScanStamp_ = INVALID_TIMESTAMP;
    std::optional<PlannerOutput> lastSinglePlan_;

    /** Returns false if a plan is already being computed. */
    bool begin_planning();

    void control_loop();
    void control_step();
    void set_status(const std::string& s);

    /** Runs TPS_Astar from `start` to `goal`. */
    PlannerOutput plan(
        const SE2_KinState& start, const SE2orR2_KinState& goal,
        std::vector<CostEvaluator::Ptr>* costEvaluatorsOut = nullptr);

    PlannerOutput plan_single(
        const SE2_KinState& start, const SE2orR2_KinState& goal);

    /** Converts a successful plan into a reference path (map frame). */
    Trajectory plan_to_reference_path(
        const PlannerOutput& plan, double targetSpeed) const;

    void set_reference_path(const Trajectory& path);

    void viz_replace(const mrpt::viz::CSetOfObjects::Ptr& obj);
    void viz_reference_path(const Trajectory& path);
    void viz_follower_state(const TrajectoryFollower::Output& out);
};

}  // namespace mpp
