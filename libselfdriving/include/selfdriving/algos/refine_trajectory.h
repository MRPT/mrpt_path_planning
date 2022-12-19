/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/data/MotionPrimitivesTree.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>

namespace selfdriving
{
/**
 * Takes a sequence of N states (the inPath) and the N-1 edges in between
 * them, and recalculate the PTG parameters of all edges using the exact poses
 * in the path nodes.
 */
void refine_trajectory(
    const MotionPrimitivesTreeSE2::path_t&    inPath,
    MotionPrimitivesTreeSE2::edge_sequence_t& edgesToRefine,
    const TrajectoriesAndRobotShape&          ptgInfo);

/// \overload taking `std::vector` of nodes and edges, instead of `std::list`
void refine_trajectory(
    const std::vector<MotionPrimitivesTreeSE2::node_t>& inPath,
    std::vector<MotionPrimitivesTreeSE2::edge_t>&       edgesToRefine,
    const TrajectoriesAndRobotShape&                    ptgInfo);

}  // namespace selfdriving
