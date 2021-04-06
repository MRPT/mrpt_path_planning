/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <selfdriving/MoveEdgeSE2_TPS.h>

#include <functional>

namespace selfdriving
{
using CostEvaluator = std::function<double(const MoveEdgeSE2_TPS&)>;
}  // namespace selfdriving
