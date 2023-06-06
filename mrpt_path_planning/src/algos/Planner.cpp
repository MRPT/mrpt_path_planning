/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/Planner.h>

using namespace mpp;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(Planner, mrpt::rtti::CObject, mpp)

Planner::~Planner() = default;

cost_t Planner::cost_path_segment(const MoveEdgeSE2_TPS& edge) const
{
    // Base cost: distance
    // cost_t c = edge.ptgDist;
    cost_t c = edge.estimatedExecTime;

    // Additional optional cost evaluators:
    for (const auto& ce : costEvaluators_)
    {
        ASSERT_(ce);
        c += (*ce)(edge);
    }

    return c;
}
