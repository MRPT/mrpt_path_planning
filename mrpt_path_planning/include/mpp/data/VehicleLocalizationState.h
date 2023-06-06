/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/system/datetime.h>

#include <optional>
#include <string>

namespace mpp
{
/** Data returned by VehicleMotionInterface::get_localization() */
struct VehicleLocalizationState
{
    /** Set to true if data could be retrieved from the robot system
     * successfully, set to false upon any critical error that made it
     * impossible to fill in the data fields in this structure. */
    bool valid = false;

    /** The current robot pose (typically from a mapping/localization module),
     * in global map coordinates. (x,y: meters, phi: radians) */
    mrpt::math::TPose2D pose;

    /** The timestamp for the read pose.
     * Use mrpt::system::now() unless you have something more accurate. */
    mrpt::system::TTimeStamp timestamp = INVALID_TIMESTAMP;

    /** Optional: if available, place here the (x,y,phi) covariance matrix
     * representing the uncertainty in the pose localization from the
     * mapping/particle filter system.
     * It could be used by the plan executor to take uncertainties into account
     * or to stop the robot if localization accuracy drops below a certain
     * required accuracy.
     */
    std::optional<mrpt::math::CMatrixDouble33> poseCov;

    /** ID of the coordinate frame for pose. Default if not
       modified is "map". [Only for possible future support for submapping] */
    std::string frame_id = "map";
};

}  // namespace mpp
