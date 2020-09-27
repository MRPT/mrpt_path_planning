/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>

namespace selfdrive
{
struct SE2_KinState
{
    SE2_KinState() = default;

    mrpt::math::TPose2D  pose{0, 0, 0};  //!< global pose
    mrpt::math::TTwist2D vel{0, 0, 0};  //!< global velocity

    std::string asString() const;
};

}  // namespace selfdrive
