/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/SE2_KinState.h>
#include <mrpt/math/CMatrixDynamic.h>

using namespace mpp;

std::string SE2_KinState::asString() const
{
    using namespace std::string_literals;

    return "p="s + pose.asString() + " v="s + vel.asString();
}

std::string SE2orR2_KinState::asString() const
{
    using namespace std::string_literals;
    std::string ret;

    if (state.isPose())
        ret = "pose="s + state.pose().asString();
    else if (state.isPoint())
        ret = "point="s + state.point().asString();
    else
        ret = "state=(undefined)"s;

    ret += " v="s + vel.asString();

    return ret;
}

SE2_KinState SE2orR2_KinState::asSE2KinState() const
{
    SE2_KinState s;
    s.vel = vel;

    if (state.isPoint())
    {
        // point:
        s.pose = {state.point().x, state.point().y, .0};
    }
    else if (state.isPose())
    {
        // pose:
        s.pose = state.pose();
    }
    else
    {
        THROW_EXCEPTION("Called with undefined state.");
    }

    return s;
}

PoseOrPoint PoseOrPoint::FromString(const std::string& s)
{
    mrpt::math::CMatrixDouble m;
    if (!m.fromMatlabStringFormat(s))
        THROW_EXCEPTION_FMT(
            "Malformed expression in FromString, s=\"%s\"", s.c_str());

    ASSERTMSG_(
        m.rows() == 1 && (m.cols() == 2 || m.cols() == 3),
        "Wrong size of vector in FromString (expected 1x2 or 1x3)");

    if (m.cols() == 2)
    {
        // point:
        return {mrpt::math::TPoint2D(m(0, 0), m(0, 1))};
    }
    else
    {
        // pose:
        return {mrpt::math::TPose2D(m(0, 0), m(0, 1), mrpt::DEG2RAD(m(0, 2)))};
    }
}

std::string PoseOrPoint::asString() const
{
    if (isPoint())
        return point().asString();
    else if (isPose())
        return pose().asString();
    else
        return {"(Undefined!)"};
}
