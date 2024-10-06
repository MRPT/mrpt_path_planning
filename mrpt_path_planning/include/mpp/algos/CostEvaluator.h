/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/version.h>

// fwd decl:
namespace mrpt::maps
{
class COccupancyGridMap2D;
}

namespace mpp
{
class CostEvaluator : public mrpt::rtti::CObject
{
#if MRPT_VERSION < 0x020e00
    DEFINE_VIRTUAL_MRPT_OBJECT(CostEvaluator)
#else
    DEFINE_VIRTUAL_MRPT_OBJECT(CostEvaluator, mpp)
#endif

   public:
    CostEvaluator() = default;
    virtual ~CostEvaluator();

    /** Evaluate cost of move-tree edge */
    virtual double operator()(const MoveEdgeSE2_TPS& edge) const = 0;

    // Default: empty viz
    virtual mrpt::opengl::CSetOfObjects::Ptr get_visualization() const;

    // Default: empty grid. Used mostly for ROS visualization interface
    virtual std::shared_ptr<mrpt::maps::COccupancyGridMap2D>
        get_visualization_as_grid() const;
};

}  // namespace mpp
