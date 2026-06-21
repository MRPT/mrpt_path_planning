/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/TPS_Astar.h>

namespace mpp
{
/**
 * Bidirectional (forward + backward) variant of TPS_Astar.
 *
 * It searches simultaneously from the start (forward lattice) and from the
 * goal (backward lattice). The two frontiers expand until they settle a node
 * in the same SE(2) lattice cell; the two half-paths are then stitched and a
 * final `refine_trajectory()` pass absorbs the sub-cell discretization gap at
 * the meeting point, exactly as the forward-only planner already relies on
 * `refine_trajectory()` after grid snapping.
 *
 * The key enabling fact is that PTG primitives are SE(2)-invariant: the same
 * trajectory set forward-simulated from a node is used to enumerate the
 * *predecessors* of a node by composing the inverse of each precomputed
 * relative pose. Collision checking of a backward edge is therefore identical
 * to a forward edge (it is still a real forward PTG arc driven from the
 * predecessor), so the collision-grid machinery is reused verbatim.
 *
 * ## Restrictions of this first implementation
 *  - **C-PTG (piecewise-constant velocity) only.** For speed-trimmable /
 *    ramped PTGs the velocity at the meeting point is not guaranteed to match;
 *    such PTGs are accepted but a warning is emitted, and the constant-velocity
 *    modeling choice of the paper is assumed.
 *  - The internal A* core runs at `heuristic_epsilon = 1.0` (exact A*),
 *    regardless of the parameter value, so the meeting/termination rule keeps
 *    a clean optimality bound. Weighted-A* for the bidirectional core is left
 *    as a follow-up.
 *
 * Both the R(2) (point, any-heading) and SE(2) (pose, with heading) goal modes
 * are supported. For an R(2) goal the backward tree is seeded with one root per
 * yaw bin of the goal cell (a multi-source backward search).
 */
class TPS_Astar_Bidir : public TPS_Astar
{
    DEFINE_MRPT_OBJECT(TPS_Astar_Bidir, mpp)

   public:
    TPS_Astar_Bidir();
    virtual ~TPS_Astar_Bidir() = default;

    PlannerOutput plan(const PlannerInput& in) override;

   private:
    enum class Direction
    {
        Forward,
        Backward
    };

    /** Backward neighbor generation: enumerate the feasible *predecessors* of
     * the node `from` (i.e. the poses `p` from which a forward PTG arc lands on
     * `from`, collision-free). Mirrors find_feasible_paths_to_neighbors() but
     * with the predecessor pose composition and the obstacle anchor at `p`.
     * Each returned entry uses the same convention as the forward generator:
     * `neighborPose = from.pose (+) relReconstrPose` yields the predecessor. */
    list_paths_to_neighbors_t find_feasible_predecessors(
        const Node& from, const TrajectoriesAndRobotShape& trs,
        const SE2orR2_KinState&                         startState,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                     MAX_XY_OBSTACLES_CLIPPING_DIST,
        const mrpt::math::TPose2D& worldBboxMin,
        const mrpt::math::TPose2D& worldBboxMax);
};

}  // namespace mpp
