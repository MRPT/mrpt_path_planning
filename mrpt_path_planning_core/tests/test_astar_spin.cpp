/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Integration tests for TPS_Astar with rotations in place (DiffDrive_Spin PTG)
 * for a differential-drive robot, together with circular-arc C-PTGs.
 *
 * Verified behaviors:
 *  - A heading change at a fixed position is a single, time-optimal rotation.
 *  - Turning around at the end of a dead-end corridor too narrow for arcs.
 *  - Turning a tight corner (L-shaped corridor) by rotating in place.
 *  - A goal behind the robot, with forward motions only, is reached by
 *    rotating to face it.
 *  - The spin shot (rotate, drive straight, rotate) reaches a pose goal
 *    exactly.
 *  - No rotation in place in open space, where arcs are faster.
 *  - Where the footprint cannot rotate, no rotation is ever planned, as
 *    checked by an exact footprint sweep against the obstacles.
 *  - Paths with rotations can be refined and sampled into trajectories.
 *  - The bidirectional planner also turns around in a dead end.
 *  - The SE(2) heuristic is consistent along rotation edges.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/TPS_Astar_Bidir.h>
#include <mpp/algos/path_collision_check.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mpp/ptgs/DiffDrive_Spin.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/wrap2pi.h>

#include <cmath>
#include <string>
#include <variant>
#include <vector>

namespace
{
using mrpt::math::TPose2D;

// Robot: 0.5 x 0.36 m rectangle, rotating about a point 0.2 m from its back:
// circumscribed radius hypot(0.30, 0.18) = 0.35 m. C-PTGs: min turning radius
// 1 / (60 deg/s) ~ 0.95 m.
const std::string kShape = R"cfg(
RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

const std::string kHeader = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0
)cfg";

std::string cPtg(int idx, int K)
{
    const std::string p = "PTG" + std::to_string(idx) + "_";
    return p + "Type        = mpp::ptg::DiffDrive_C\n" + p +
           "resolution  = 0.10\n" + p + "refDistance = 4.0\n" + p +
           "num_paths   = 41\n" + p + "v_max_mps   = 1.0\n" + p +
           "w_max_dps   = 60.0\n" + p + "K           = " + std::to_string(K) +
           "\n";
}

std::string spinPtg(int idx)
{
    const std::string p = "PTG" + std::to_string(idx) + "_";
    return p + "Type        = mpp::ptg::DiffDrive_Spin\n" + p +
           "resolution  = 0.05\n" + p + "w_max_dps   = 60.0\n";
}

// Forward and reverse C-PTGs, and rotations in place (PTG index 2).
const std::string kFwdRevSpin = kHeader + "PTG_COUNT = 3\n" + cPtg(0, 1) +
                                cPtg(1, -1) + spinPtg(2) + kShape;
constexpr int kSpinIdxFwdRevSpin = 2;

// Forward C-PTG and rotations in place (PTG index 1).
const std::string kFwdSpin =
    kHeader + "PTG_COUNT = 2\n" + cPtg(0, 1) + spinPtg(1) + kShape;
constexpr int kSpinIdxFwdSpin = 1;

// Forward and reverse C-PTGs only.
const std::string kFwdRev =
    kHeader + "PTG_COUNT = 2\n" + cPtg(0, 1) + cPtg(1, -1) + kShape;

const double kWmax = mrpt::DEG2RAD(60.0);

void addSegment(
    mrpt::maps::CSimplePointsMap& pts, double x0, double y0, double x1,
    double y1)
{
    const double len = std::hypot(x1 - x0, y1 - y0);
    const int    n   = std::max(1, static_cast<int>(std::ceil(len / 0.05)));
    for (int i = 0; i <= n; i++)
    {
        const double t = double(i) / n;
        pts.insertPoint(
            static_cast<float>(x0 + t * (x1 - x0)),
            static_cast<float>(y0 + t * (y1 - y0)), 0.0f);
    }
}

mpp::PlannerInput buildInput(
    const std::string& ptgCfg, const TPose2D& start,
    const mpp::PoseOrPoint& goal, const TPose2D& bboxMin,
    const TPose2D&                           bboxMax,
    const mrpt::maps::CSimplePointsMap::Ptr& obs = nullptr)
{
    mrpt::config::CConfigFileMemory cfg(ptgCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    in.stateStart.pose = start;
    in.stateGoal.state = goal;
    in.worldBboxMin    = bboxMin;
    in.worldBboxMax    = bboxMax;
    if (obs)
    {
        in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));
    }
    return in;
}

mpp::TPS_Astar buildPlanner()
{
    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.10;
    planner.params_.grid_resolution_yaw             = mrpt::DEG2RAD(7.5);
    planner.params_.max_ptg_trajectories_to_explore = 20;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.5, 3.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 20.0;
    planner.params_.SE2_metricAngleWeight           = 1.0;
    planner.params_.heuristic_heading_weight        = 0.1;
    return planner;
}

std::vector<const mpp::MoveEdgeSE2_TPS*> solutionEdges(
    const mpp::PlannerOutput& out)
{
    std::vector<const mpp::MoveEdgeSE2_TPS*> r;
    if (!out.success || !out.goalNodeId) { return r; }
    const auto [nodes, edges] = out.motionTree.backtrack_path(*out.goalNodeId);
    for (const auto* e : edges)
    {
        if (e != nullptr) { r.push_back(e); }
    }
    return r;
}

size_t countEdgesOfPtg(const mpp::PlannerOutput& out, int ptgIndex)
{
    size_t n = 0;
    for (const auto* e : solutionEdges(out))
    {
        if (e->ptgIndex == ptgIndex) { n++; }
    }
    return n;
}

// Exact check of the footprint against the obstacle points, at every PTG step
// of the edge (rotations of 1 ms at most 0.06 deg; motions of 1 mm).
bool edgeIsCollisionFree(
    const mpp::MoveEdgeSE2_TPS& e, const mpp::PlannerInput& in,
    const mrpt::maps::CSimplePointsMap& obs)
{
    const auto& ptg   = in.ptgs.ptgs.at(e.ptgIndex);
    const auto  shape = std::get<mrpt::math::TPolygon2D>(in.ptgs.robotShape);
    const auto& xs    = obs.getPointsBufferRef_x();
    const auto& ys    = obs.getPointsBufferRef_y();
    for (uint32_t s = 0; s <= e.ptgStepIndex; s++)
    {
        const auto p = e.stateFrom.pose + ptg->getPathPose(e.ptgPathIndex, s);
        for (size_t i = 0; i < xs.size(); i++)
        {
            const auto local = TPose2D(xs[i], ys[i], 0) - p;
            if (shape.contains({local.x, local.y})) { return false; }
        }
    }
    return true;
}

void expectPathCollisionFree(
    const mpp::PlannerOutput& out, const mpp::PlannerInput& in,
    const mrpt::maps::CSimplePointsMap::Ptr& obs)
{
    const auto edges = solutionEdges(out);
    ASSERT_FALSE(edges.empty());
    for (size_t i = 0; i < edges.size(); i++)
    {
        EXPECT_TRUE(edgeIsCollisionFree(*edges[i], in, *obs)) << "edge #" << i;
    }
    const std::vector<mrpt::maps::CPointsMap::Ptr> obsList = {obs};
    EXPECT_FALSE(mpp::first_colliding_edge(edges, in.ptgs, obsList));
}

// Dead-end corridor along +x, closed at x = xEnd, with walls at +-halfWidth.
mrpt::maps::CSimplePointsMap::Ptr deadEndCorridor(
    double halfWidth, double x0, double xEnd)
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    addSegment(*pts, x0, halfWidth, xEnd, halfWidth);
    addSegment(*pts, x0, -halfWidth, xEnd, -halfWidth);
    addSegment(*pts, xEnd, -halfWidth, xEnd, halfWidth);
    return pts;
}
}  // namespace

TEST(AstarSpin, InPlaceHeadingChangeIsOneRotation)
{
    auto       planner = buildPlanner();
    const auto in      = buildInput(
             kFwdRevSpin, {0, 0, 0}, TPose2D{0, 0, M_PI / 2}, {-3, -3, -M_PI},
             {3, 3, M_PI});
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);

    const auto edges = solutionEdges(out);
    ASSERT_EQ(edges.size(), 1U);
    EXPECT_EQ(edges[0]->ptgIndex, kSpinIdxFwdRevSpin);
    EXPECT_NEAR(out.pathCost, (M_PI / 2) / kWmax, 0.01);
    const auto end =
        edges[0]->stateFrom.pose +
        in.ptgs.ptgs.at(edges[0]->ptgIndex)
            ->getPathPose(edges[0]->ptgPathIndex, edges[0]->ptgStepIndex);
    EXPECT_NEAR(end.x, 0.0, 1e-9);
    EXPECT_NEAR(end.y, 0.0, 1e-9);
    EXPECT_NEAR(mrpt::math::angDistance(end.phi, M_PI / 2), 0.0, 1e-3);

    // Without rotations in place, the same goal does not cost less (a
    // three-point turn, nearly as fast if stopping times are neglected):
    auto       plannerC = buildPlanner();
    const auto inC      = buildInput(
             kFwdRev, {0, 0, 0}, TPose2D{0, 0, M_PI / 2}, {-3, -3, -M_PI},
             {3, 3, M_PI});
    const auto outC = plannerC.plan(inC);
    ASSERT_TRUE(outC.success);
    EXPECT_GE(outC.pathCost, out.pathCost);
}

TEST(AstarSpin, TurnAroundInDeadEnd)
{
    // 0.9 m wide: the 0.7 m rotation circle fits, arcs of radius 0.95 m don't.
    const auto obs   = deadEndCorridor(0.45, -2.0, 3.0);
    const auto start = TPose2D{2.2, 0, 0};
    const auto goal  = TPose2D{2.2, 0, M_PI};

    auto       planner = buildPlanner();
    const auto in      = buildInput(
             kFwdRevSpin, start, goal, {-2, -0.45, -M_PI}, {3, 0.45, M_PI}, obs);
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    EXPECT_GE(countEdgesOfPtg(out, kSpinIdxFwdRevSpin), 1U);
    EXPECT_LT(out.pathCost, 1.5 * M_PI / kWmax);
    expectPathCollisionFree(out, in, obs);

    // Without rotations in place, it is not possible, or much more costly:
    auto plannerC                           = buildPlanner();
    plannerC.params_.maximumComputationTime = 5.0;
    const auto inC                          = buildInput(
                                 kFwdRev, start, goal, {-2, -0.45, -M_PI}, {3, 0.45, M_PI}, obs);
    const auto outC = plannerC.plan(inC);
    EXPECT_TRUE(!outC.success || outC.pathCost > 2 * out.pathCost)
        << outC.pathCost;
}

TEST(AstarSpin, TightCorner)
{
    // L-shaped corridor, 0.9 m wide: along +x, then up along +y at x=2.
    auto obs = mrpt::maps::CSimplePointsMap::Create();
    addSegment(*obs, -1.0, -0.45, 2.45, -0.45);  // bottom
    addSegment(*obs, -1.0, 0.45, 1.55, 0.45);  // top
    addSegment(*obs, 2.45, -0.45, 2.45, 3.0);  // right
    addSegment(*obs, 1.55, 0.45, 1.55, 3.0);  // left
    addSegment(*obs, -1.0, -0.45, -1.0, 0.45);  // closed behind

    auto       planner = buildPlanner();
    const auto in      = buildInput(
             kFwdRevSpin, {0, 0, 0}, mrpt::math::TPoint2D{2.0, 2.5},
             {-1, -0.45, -M_PI}, {2.45, 3, M_PI}, obs);
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    EXPECT_GE(countEdgesOfPtg(out, kSpinIdxFwdRevSpin), 1U);
    expectPathCollisionFree(out, in, obs);

    // The path can be refined and sampled: during rotations, the position
    // does not change.
    auto [path, edges] = out.motionTree.backtrack_path(*out.goalNodeId);
    mpp::refine_trajectory(path, edges, in.ptgs);
    const auto traj = mpp::plan_to_trajectory(edges, in.ptgs, 0.05);
    ASSERT_GT(traj.size(), 10U);
    double tPrev = -1;
    for (const auto& [t, s] : traj)
    {
        EXPECT_GT(t, tPrev);
        tPrev = t;
        if (s.ptgIndex == kSpinIdxFwdRevSpin)
        {
            EXPECT_EQ(s.state.vel.vx, 0.0);
            EXPECT_EQ(s.state.vel.vy, 0.0);
        }
    }
}

TEST(AstarSpin, FacesGoalBehindWithForwardMotionsOnly)
{
    const auto obs = deadEndCorridor(0.45, -3.0, 1.0);

    auto       planner = buildPlanner();
    const auto in      = buildInput(
             kFwdSpin, {0, 0, 0}, mrpt::math::TPoint2D{-2.0, 0.0},
             {-3, -0.45, -M_PI}, {1, 0.45, M_PI}, obs);
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    const auto edges = solutionEdges(out);
    ASSERT_GE(edges.size(), 2U);
    EXPECT_EQ(edges.front()->ptgIndex, kSpinIdxFwdSpin);
    expectPathCollisionFree(out, in, obs);
}

TEST(AstarSpin, SpinShotReachesPoseGoalExactly)
{
    // Sideways displacement, same heading: forward arcs alone would need
    // loops; rotate, drive straight 1 m, and rotate back.
    const TPose2D goal{0, 1.0, 0};
    auto          planner = buildPlanner();
    const auto    in =
        buildInput(kFwdSpin, {0, 0, 0}, goal, {-3, -3, -M_PI}, {3, 3, M_PI});
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);

    // Composing the PTG motions of all edges ends at the goal itself:
    const auto edges = solutionEdges(out);
    ASSERT_FALSE(edges.empty());
    TPose2D p = in.stateStart.pose;
    for (const auto* e : edges)
    {
        p = p + in.ptgs.ptgs.at(e->ptgIndex)
                    ->getPathPose(e->ptgPathIndex, e->ptgStepIndex);
    }
    EXPECT_NEAR(p.x, goal.x, 2e-3);
    EXPECT_NEAR(p.y, goal.y, 2e-3);
    EXPECT_NEAR(mrpt::math::angDistance(p.phi, goal.phi), 0.0, 2e-3);
    EXPECT_GE(countEdgesOfPtg(out, kSpinIdxFwdSpin), 2U);
    // Rotate pi/2 twice and drive 1 m:
    EXPECT_NEAR(out.pathCost, M_PI / kWmax + 1.0, 0.05);
}

TEST(AstarSpin, NoRotationsInOpenSpaceAhead)
{
    auto planner                      = buildPlanner();
    planner.params_.heuristic_epsilon = 1.0;
    const auto in                     = buildInput(
                            kFwdRevSpin, {0, 0, 0}, mrpt::math::TPoint2D{4.0, 0.8}, {-2, -3, -M_PI},
                            {6, 3, M_PI});
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    EXPECT_EQ(countEdgesOfPtg(out, kSpinIdxFwdRevSpin), 0U);
}

TEST(AstarSpin, NoRotationWhereTheFootprintDoesNotFit)
{
    // Narrower than the footprint diagonal (0.58 m): the robot drives along
    // it, but cannot rotate by 90 degrees in any way. Small rotations fit.
    const auto obs   = deadEndCorridor(0.28, -1.0, 3.0);
    const auto start = TPose2D{1.0, 0, 0};

    auto planner                           = buildPlanner();
    planner.params_.maximumComputationTime = 5.0;
    const auto in                          = buildInput(
                                 kFwdRevSpin, start, TPose2D{1.0, 0, M_PI}, {-1, -0.28, -M_PI},
                                 {3, 0.28, M_PI}, obs);
    const auto out = planner.plan(in);
    EXPECT_FALSE(out.success);

    // No rotation edge in the whole search tree may hit the walls:
    for (const auto& [parent, children] : out.motionTree.edges_to_children)
    {
        for (const auto& c : children)
        {
            if (c.data.ptgIndex != kSpinIdxFwdRevSpin) { continue; }
            EXPECT_TRUE(edgeIsCollisionFree(c.data, in, *obs));
        }
    }
}

TEST(AstarSpin, HeuristicConsistentAlongRotations)
{
    const auto obs  = deadEndCorridor(0.45, -2.0, 3.0);
    const auto goal = TPose2D{2.2, 0, M_PI};

    auto planner                      = buildPlanner();
    planner.params_.heuristic_epsilon = 1.0;
    const auto in                     = buildInput(
                            kFwdRevSpin, {0.5, 0, 0}, goal, {-2, -0.45, -M_PI}, {3, 0.45, M_PI},
                            obs);
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);

    size_t nSpinEdges = 0;
    for (const auto& [parent, children] : out.motionTree.edges_to_children)
    {
        for (const auto& c : children)
        {
            const auto& e = c.data;
            if (e.ptgIndex != kSpinIdxFwdRevSpin) { continue; }
            nSpinEdges++;
            const double hFrom =
                planner.default_heuristic_SE2(e.stateFrom, goal);
            const double hTo = planner.default_heuristic_SE2(e.stateTo, goal);
            EXPECT_LE(hFrom, planner.cost_path_segment(e) + hTo + 1e-6);
        }
    }
    EXPECT_GT(nSpinEdges, 0U);
}

TEST(AstarSpin, BidirTurnAroundInDeadEnd)
{
    const auto obs = deadEndCorridor(0.45, -2.0, 3.0);

    mpp::TPS_Astar_Bidir planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_ = buildPlanner().params_;
    const auto in   = buildInput(
          kFwdRevSpin, {0.5, 0, 0}, TPose2D{2.2, 0, M_PI}, {-2, -0.45, -M_PI},
          {3, 0.45, M_PI}, obs);
    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    EXPECT_GE(countEdgesOfPtg(out, kSpinIdxFwdRevSpin), 1U);
    expectPathCollisionFree(out, in, obs);
}
