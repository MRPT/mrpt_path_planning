/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/lock_helper.h>
#include <selfdriving/algos/WaypointSequencer.h>

using namespace selfdriving;

WaypointSequencer::~WaypointSequencer()
{
    // stop vehicle, etc.
}

void WaypointSequencer::initialize()
{
    // Check that config_ holds all the required fields:
}

void WaypointSequencer::requestNavigation(const WaypointSequence& navRequest)
{
    //
}

void WaypointSequencer::navigationStep()
{
    //
}

void WaypointSequencer::cancel() {}
void WaypointSequencer::resume() {}
void WaypointSequencer::suspend() {}
void WaypointSequencer::resetNavError() {}

WaypointStatusSequence WaypointSequencer::getWaypointNavStatus() const
{
    // Make sure the data structure is not under modification:
    auto                   lck = mrpt::lockHelper(m_nav_waypoints_cs);
    WaypointStatusSequence ret = m_waypoint_nav_status;

    return ret;
}

void WaypointSequencer::dispatchPendingNavEvents() {}

void WaypointSequencer::updateCurrentPoseAndSpeeds() {}

void WaypointSequencer::performNavigationStepNavigating(
    bool call_virtual_nav_method)
{
}
