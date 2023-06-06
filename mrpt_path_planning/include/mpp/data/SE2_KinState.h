/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>

#include <variant>

// Include the "_deg" literal from mrpt:
#include <mrpt/core/bits_math.h>  // _deg literal
#include <mrpt/version.h>

namespace mpp
{
#if MRPT_VERSION >= 0x258
using namespace mrpt::literals;  // "_deg" literal
#else
using namespace mrpt;  // backwards compat
#endif

/** Variant wrapper holding a SE(2) pose, a R(2) (2D) point, or none (default).
 * It is used to provide a goal or waypoint state, when the heading is not
 * important and the user only wants to specify the (x,y) coordinates of the 2D
 * point.
 */
struct PoseOrPoint
{
    PoseOrPoint()  = default;
    ~PoseOrPoint() = default;

    /** Returns a point or a pose depending on the input string having the
     * format `"[x y]"` or `"[x y phi_degrees]"`, respectively.
     */
    static PoseOrPoint FromString(const std::string& s);

    PoseOrPoint(const mrpt::math::TPoint2D& p) : data_(p) {}
    PoseOrPoint(const mrpt::math::TPose2D& p) : data_(p) {}

    bool isEmpty() const { return data_.index() == 0; }
    bool isPose() const { return data_.index() == 1; }
    bool isPoint() const { return data_.index() == 2; }

    const mrpt::math::TPose2D& pose() const
    {
        return std::get<mrpt::math::TPose2D>(data_);
    }
    const mrpt::math::TPoint2D& point() const
    {
        return std::get<mrpt::math::TPoint2D>(data_);
    }

    std::string asString() const;

   private:
    std::variant<std::monostate, mrpt::math::TPose2D, mrpt::math::TPoint2D>
        data_;
};

struct SE2_KinState
{
    SE2_KinState() = default;

    mrpt::math::TPose2D  pose{0, 0, 0};  //!< global pose (x,y,phi)
    mrpt::math::TTwist2D vel{0, 0, 0};  //!< global velocity (vx,vy,omega)

    std::string asString() const;
};

struct SE2orR2_KinState
{
    SE2orR2_KinState() = default;

    PoseOrPoint          state;  //!< global pose (x,y,phi) or point(x,y)
    mrpt::math::TTwist2D vel{0, 0, 0};  //!< global velocity (vx,vy,omega)

    /** Returns a SE2_KinState exact equivalent of this object, if state is a
    pose, or transforms the point into a pose with phi=0 otherwise. */
    SE2_KinState asSE2KinState() const;

    std::string asString() const;
};

}  // namespace mpp
