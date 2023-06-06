/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/opengl/opengl_frwds.h>

#include <memory>

namespace mpp
{
struct RenderVehicleParams
{
    RenderVehicleParams() = default;

    int NUM_VERTICES = 14;  //!< for circular robot shapes.
};

struct RenderVehicleExtraResults
{
    RenderVehicleExtraResults() = default;

    double maxVehicleShapeRadius = 0;
};

/** Generates a polygon for the vehicle shape */
auto render_vehicle(
    const RobotShape& rs, mrpt::opengl::CSetOfLines& outPolygon,
    const RenderVehicleParams& rvp = {}) -> RenderVehicleExtraResults;

}  // namespace mpp
