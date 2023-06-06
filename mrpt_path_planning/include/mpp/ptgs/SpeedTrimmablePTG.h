/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <cstdlib>

namespace mpp::ptg
{
/** A PTG with a dynamic variable for speed modulation [0,1], usable in
 *  the V and W expressions.
 *
 */
class SpeedTrimmablePTG
{
   public:
    SpeedTrimmablePTG()  = default;
    ~SpeedTrimmablePTG() = default;

    double trimmableSpeed_ = 1.0;
};
}  // namespace mpp::ptg
