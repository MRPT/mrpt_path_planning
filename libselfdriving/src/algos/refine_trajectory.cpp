/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/algos/edge_interpolated_path.h>
#include <selfdriving/algos/refine_trajectory.h>
#include <selfdriving/ptgs/SpeedTrimmablePTG.h>

#include <iostream>

void selfdriving::refine_trajectory(
    MotionPrimitivesTreeSE2::path_t&          inPath,
    MotionPrimitivesTreeSE2::edge_sequence_t& inEdges,
    const TrajectoriesAndRobotShape&          ptgInfo)
{
    auto                                     outPath = inPath;
    MotionPrimitivesTreeSE2::edge_sequence_t outEdges;

    const size_t nEdges = inEdges.size();
    ASSERT_EQUAL_(inPath.size(), nEdges + 1);

    auto itPath = inPath.begin();
    auto itEdge = inEdges.begin();
    for (size_t i = 0; i < nEdges; i++, itPath++, itEdge++)
    {
        // Get edge:
        auto& edge = **itEdge;

        auto& ptg = ptgInfo.ptgs.at(edge.ptgIndex);
        ptg->updateNavDynamicState(edge.getPTGDynState());
        if (auto* ptgTrim = dynamic_cast<ptg::SpeedTrimmablePTG*>(ptg.get());
            ptgTrim)
            ptgTrim->trimmableSpeed_ = edge.ptgTrimmableSpeed;

        // get current and next nodes:
        auto itNextPath = itPath;
        itNextPath++;
        const auto& startNode = *itPath;
        auto&       endNode   = *itNextPath;

#if 0
        std::cout << "[refine_trajectory] Step #" << i << " INPUT \n     "
                  << startNode.asString() << "\n ==> " << endNode.asString()
                  << "\n";
#endif

        const auto deltaNodes = endNode.pose - startNode.pose;

        int                   newK        = -1;
        normalized_distance_t newNormDist = 0;

        const bool ok = ptg->inverseMap_WS2TP(
            deltaNodes.x, deltaNodes.y, newK, newNormDist);
        if (!ok)
        {
            std::stringstream ss;
            ss << "Assert failed: ptg->inverseMap_WS2TP() => returned "
                  "ok=false. More info:\n";
            ss << " - PTG: " << ptg->getDescription() << "\n";
            ss << " - deltaNodes: " << deltaNodes.asString() << "\n";
            ss << " - edge: " << edge.asString() << "\n";
            // THROW_EXCEPTION(ss.str());
            std::cerr << "[refine_trajectory] Warning: Could not refine this "
                         "path segment:\n"
                      << ss.str() << std::endl;
        }
        else
        {
            distance_t newDist = newNormDist * ptg->getRefDistance();

            uint32_t newPtgStep = 0;
            ptg->getPathStepForDist(newK, newDist, newPtgStep);

#if 0
        std::cout << "    Corrections: pathIndex " << edge.ptgPathIndex
                  << " => " << newK << " ptgDist:" << edge.ptgDist << " => "
                  << newDist << "\n";
#endif

            edge.ptgPathIndex = newK;
            edge.ptgDist      = newDist;

            // Update interpolated path:
            edge_interpolated_path(edge, ptgInfo, deltaNodes, newPtgStep);
        }
    }
}
