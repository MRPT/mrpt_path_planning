/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <functional>  // reference_wrapper<>
#include <optional>

namespace mpp
{
/** Wrapper to a const reference to T */
template <typename T>
using const_ref_t = std::optional<std::reference_wrapper<const T>>;

}  // namespace mpp
