/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/SE2_KinState.h>
#include <mpp/data/basic_types.h>

#include <map>

namespace mpp
{
struct trajectory_state_t
{
    trajectory_state_t() = default;

    SE2_KinState       state;
    ptg_index_t        ptgIndex     = 0;
    trajectory_index_t ptgPathIndex = 0;
    uint32_t           ptgStep      = 0;
};

using trajectory_t = std::map<duration_seconds_t, trajectory_state_t>;

}  // namespace mpp
