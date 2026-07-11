/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/robot_shape_sampling.h>

#include <algorithm>
#include <cmath>

std::vector<mrpt::math::TPoint2D> mpp::footprintSamplePoints(
    const RobotShape& shape, double resolution, std::size_t maxSamples)
{
    std::vector<mrpt::math::TPoint2D> pts;
    const double                      res = std::max(0.01, resolution);

    if (const auto* poly = std::get_if<mrpt::math::TPolygon2D>(&shape))
    {
        const auto&       v = *poly;
        const std::size_t n = v.size();
        if (n < 2) return pts;

        double perimeter = 0;
        for (std::size_t i = 0; i < n; i++)
            perimeter += (v[(i + 1) % n] - v[i]).norm();

        // Step at the grid resolution, coarsened if needed to stay under the
        // cap.
        const double step = std::max(res, perimeter / maxSamples);

        for (std::size_t i = 0; i < n; i++)
        {
            const auto&  a   = v[i];
            const auto&  b   = v[(i + 1) % n];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            const int    nSeg =
                std::max(1, static_cast<int>(std::ceil(len / step)));
            for (int k = 0; k < nSeg;
                 k++)  // include a, exclude b (next edge's a)
            {
                const double t = static_cast<double>(k) / nSeg;
                pts.emplace_back(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
            }
        }
    }
    else if (const auto* radius = std::get_if<mpp::robot_radius_t>(&shape))
    {
        const double r = *radius;
        if (r <= 0) return pts;
        const double step = std::max(res, 2 * M_PI * r / maxSamples);
        const int    nSeg =
            std::max(8, static_cast<int>(std::ceil(2 * M_PI * r / step)));
        for (int k = 0; k < nSeg; k++)
        {
            const double a = 2 * M_PI * k / nSeg;
            pts.emplace_back(r * std::cos(a), r * std::sin(a));
        }
    }
    // std::monostate -> empty
    return pts;
}
