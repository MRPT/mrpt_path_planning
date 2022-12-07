/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/interfaces/TargetApproachController.h>

using namespace selfdriving;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(
    TargetApproachController, mrpt::rtti::CObject, selfdriving)

TargetApproachController::~TargetApproachController() = default;
