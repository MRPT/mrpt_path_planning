/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/data/MotionPrimitivesTree.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>

namespace selfdriving
{
void refine_trajectory(
    MotionPrimitivesTreeSE2::path_t&          inPath,
    MotionPrimitivesTreeSE2::edge_sequence_t& inEdges,
    const TrajectoriesAndRobotShape&          ptgInfo);

}
