/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>

namespace mpp
{
static inline bool within_bbox(
    const mrpt::math::TPose2D& p, const mrpt::math::TPose2D& max,
    const mrpt::math::TPose2D& min)
{
    return p.x < max.x && p.y < max.y && p.phi < max.phi + 1e-6 &&  //
           p.x > min.x && p.y > min.y && p.phi > min.phi - 1e-6;
}

static inline bool within_bbox(
    const mrpt::math::TPoint2D& p, const mrpt::math::TPose2D& max,
    const mrpt::math::TPose2D& min)
{
    return p.x < max.x && p.y < max.y &&  //
           p.x > min.x && p.y > min.y;
}

}  // namespace mpp
