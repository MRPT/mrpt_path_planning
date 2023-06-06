/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluator.h>

using namespace mpp;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CostEvaluator, mrpt::rtti::CObject, mpp)

CostEvaluator::~CostEvaluator() = default;

mrpt::opengl::CSetOfObjects::Ptr CostEvaluator::get_visualization() const
{
    // Default: empty viz
    auto glObj = mrpt::opengl::CSetOfObjects::Create();
    glObj->setName("CostEvaluator.default");
    return glObj;
}
