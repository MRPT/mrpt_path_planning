/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/SE2_KinState.h>

using namespace selfdriving;

std::string SE2_KinState::asString() const
{
    return std::string("p=") + pose.asString() + std::string(" v=") +
           vel.asString();
}
