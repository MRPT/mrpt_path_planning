/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/interfaces/VehicleMotionInterface.h>

using namespace selfdriving;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(VehicleMotionInterface, mrpt::rtti::CObject, selfdriving)

VehicleMotionInterface::~VehicleMotionInterface() = default;

