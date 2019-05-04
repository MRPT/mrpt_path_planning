/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

/** @file mrpt_1or2.h Cross-compatibility header for mrpt-1.5 and mrpt-2. */

#include <mrpt/version.h>

#if MRPT_VERSION>=0x199
#include <mrpt/system/CTimeLogger.h>
namespace selfdrive
{
using mrpt::system::CTimeLogger;
using mrpt::system::CTimeLoggerEntry;
}
#else
#include <mrpt/utils/CTimeLogger.h>
namespace selfdrive
{
using mrpt::utils::CTimeLogger;
using mrpt::utils::CTimeLoggerEntry;
}
#endif
