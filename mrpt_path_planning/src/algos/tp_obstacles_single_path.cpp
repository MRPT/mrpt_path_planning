/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/tp_obstacles_single_path.h>

mpp::distance_t mpp::tp_obstacles_single_path(
    const trajectory_index_t      tp_space_k_direction,
    const mrpt::maps::CPointsMap& localObstacles, const ptg_t& ptg)
{
    MRPT_START
    // Take "k_rand"s and "distances" such that the collision hits the
    // obstacles
    // in the "grid" of the given PT
    // --------------------------------------------------------------------
    size_t       nObs;
    const float *obs_xs, *obs_ys, *obs_zs;
    localObstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

    // Init obs ranges:
    distance_t out_TPObstacle_k = 0;
    ptg.initTPObstacleSingle(tp_space_k_direction, out_TPObstacle_k);

    for (size_t obs = 0; obs < nObs; obs++)
    {
        const float ox = obs_xs[obs];
        const float oy = obs_ys[obs];

        ptg.updateTPObstacleSingle(
            ox, oy, tp_space_k_direction, out_TPObstacle_k);
    }

    // Leave distances in out_TPObstacles un-normalized, so they
    // just represent real distances in "pseudo-meters".
    return out_TPObstacle_k;

    MRPT_END
}
