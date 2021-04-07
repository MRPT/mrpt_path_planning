/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/rtti/CObject.h>
#include <selfdriving/MoveEdgeSE2_TPS.h>

#include <functional>

namespace selfdriving
{
class CostEvaluator : public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(CostEvaluator)

   public:
    CostEvaluator() = default;
    virtual ~CostEvaluator();

    /** Evaluate cost of move-tree edge */
    virtual double operator()(const MoveEdgeSE2_TPS& edge) const = 0;
};

}  // namespace selfdriving
