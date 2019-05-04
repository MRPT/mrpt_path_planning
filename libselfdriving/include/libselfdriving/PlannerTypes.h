/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <memory>

namespace selfdrive
{
struct RobotShape
{
	mrpt::math::TPolygon2D robot_shape;  //!< 2D robot shape
	double                 robot_radius{-1.0};  //!< Radius of circ. robot
};

struct SE2_KinState
{
	mrpt::math::TPose2D  pose{0, 0, 0};
	mrpt::math::TTwist2D vel{0, 0, 0};
};

class ObstacleSource
{
   public:
	using Ptr = std::shared_ptr<ObstacleSource>;

	ObstacleSource() = default;
	virtual ~ObstacleSource();

	virtual mrpt::maps::CSimplePointsMap::Ptr obstacles(
		mrpt::system::TTimeStamp t = mrpt::system::TTimeStamp()) = 0;

	virtual bool dynamic() const { return false; }
};

struct PlannerInput
{
	RobotShape          robot_shape;
	SE2_KinState        state_start, state_goal;
	ObstacleSource::Ptr obstacles;
};

}  // namespace selfdrive
