/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/algos/Planner.h>

using namespace selfdriving;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(Planner, mrpt::rtti::CObject, selfdriving)

Planner::~Planner() = default;
