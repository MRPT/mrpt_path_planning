/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <libselfdriving/MotionPrimitivesTree.h>
#include <libselfdriving/SE2_KinState.h>
#include <libselfdriving/ptg_t.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <limits>
#include <memory>
#include <vector>

namespace selfdrive
{
struct RobotShape
{
    mrpt::math::TPolygon2D robot_shape;  //!< 2D robot shape
    double                 robot_radius{-1.0};  //!< Radius of circ. robot
};

class TrajectoriesAndRobotShape
{
   public:
    TrajectoriesAndRobotShape()  = default;
    ~TrajectoriesAndRobotShape() = default;

    bool initialized() const { return initialized_; }
    void clear();

    void initFromConfigFile(
        mrpt::config::CConfigFileBase& cfg, const std::string& section);
    void initFromYAML(const mrpt::containers::yaml& node);

    std::vector<std::shared_ptr<ptg_t>> ptgs;  //!< Allowed movement sets
    RobotShape                          robotShape;

   private:
    bool initialized_ = false;
};

struct PlannerInput
{
    RobotShape          robot_shape;
    SE2_KinState        state_start, state_goal;
    mrpt::math::TPose2D world_bbox_min, world_bbox_max;  //!< World bounding box
    ObstacleSource::Ptr obstacles;
    double              min_step_len{0.25};  //!< [meters]
    TrajectoriesAndRobotShape ptgs;
};

/** The output of the path planner */
struct NavPlan
{
    PlannerInput originalInput;

    bool success = false;

    /** Time spent (in secs) */
    double computationTime = 0;

    /** Distance from best found path to goal */
    double goalDistance = std::numeric_limits<double>::max();

    /** Total cost of the best found path (cost; Euclidean distance) */
    double pathCost = std::numeric_limits<double>::max();

    /** The ID of the best target node in the tree */
    mrpt::graphs::TNodeID best_goal_node_id = INVALID_NODEID;

    /** The set of target nodes within an acceptable distance to target
     * (including `best_goal_node_id` and others) */
    std::set<mrpt::graphs::TNodeID> acceptable_goal_node_ids;

    /** The generated motion tree that explores free space starting at "start"
     */
    MotionPrimitivesTreeSE2 motionTree;
};

}  // namespace selfdrive
