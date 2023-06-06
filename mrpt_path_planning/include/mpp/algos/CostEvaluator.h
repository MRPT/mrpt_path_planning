/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/rtti/CObject.h>

#include <functional>

namespace mpp
{
class CostEvaluator : public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(CostEvaluator)

   public:
    CostEvaluator() = default;
    virtual ~CostEvaluator();

    /** Evaluate cost of move-tree edge */
    virtual double operator()(const MoveEdgeSE2_TPS& edge) const = 0;

    // Default: empty viz
    virtual mrpt::opengl::CSetOfObjects::Ptr get_visualization() const;
};

}  // namespace mpp
