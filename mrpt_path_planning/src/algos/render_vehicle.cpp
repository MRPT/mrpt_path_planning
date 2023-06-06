/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/render_vehicle.h>
#include <mrpt/opengl/CSetOfLines.h>

using namespace mpp;

/** Generates a polygon for the vehicle shape */
auto mpp::render_vehicle(
    const RobotShape& rs, mrpt::opengl::CSetOfLines& outPolygon,
    const RenderVehicleParams& rvp) -> RenderVehicleExtraResults
{
    RenderVehicleExtraResults res;

    outPolygon.clear();

    if (auto pPoly = std::get_if<mrpt::math::TPolygon2D>(&rs); pPoly)
    {
        const auto& poly = *pPoly;
        outPolygon.appendLine(poly[0].x, poly[0].y, 0, poly[1].x, poly[1].y, 0);
        for (size_t i = 2; i <= poly.size(); i++)
        {
            const size_t idx = i % poly.size();
            mrpt::keep_max(res.maxVehicleShapeRadius, poly[idx].norm());
            outPolygon.appendLineStrip(poly[idx].x, poly[idx].y, 0);
        }
    }
    else if (auto pRadius = std::get_if<double>(&rs); pRadius)
    {
        const double R            = *pRadius;
        const int    NUM_VERTICES = rvp.NUM_VERTICES;
        for (int i = 0; i <= NUM_VERTICES; i++)
        {
            const size_t idx  = i % NUM_VERTICES;
            const size_t idxn = (i + 1) % NUM_VERTICES;
            const double ang  = idx * 2 * M_PI / (NUM_VERTICES - 1);
            const double angn = idxn * 2 * M_PI / (NUM_VERTICES - 1);
            outPolygon.appendLine(
                R * cos(ang), R * sin(ang), 0, R * cos(angn), R * sin(angn), 0);
        }
        outPolygon.appendLine(0, R, 0, 0, -R, 0);
        outPolygon.appendLine(0, R, 0, R, 0, 0);
        outPolygon.appendLine(0, -R, 0, R, 0, 0);

        mrpt::keep_max(res.maxVehicleShapeRadius, R);
    }
    else
    {
        THROW_EXCEPTION("Invalid vehicle shape variant<> type.");
    }

    return res;
}
