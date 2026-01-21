^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_path_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2026-01-21)
------------------
* fix build against newer mvsim API
* Contributors: Jose Luis Blanco-Claraco

0.2.4 (2025-12-19)
------------------
* Fix build for mrpt >=2.15.3
* Contributors: Jose Luis Blanco-Claraco

0.2.3 (2025-11-10)
------------------
* Fix usage of new mrpt 2.15.0 API
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2025-07-22)
------------------
* README: update badges for active ROS distributions
* modernize C++, add [[nodiscard]]
* Update README.md ROS badges
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2024-10-14)
------------------
* costmaps for rviz API
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2024-10-05)
------------------
* Add support for new PTGs with internalState in MRPT >=2.14.2
* viz animations: draw vehicle frame for visual reference
* interpolate(): fix accumulated error along trajectories
* cli and viz: interpolate trajectory
* cli: add flags to control obstacles bbox and clipping
* ObstacleSource virtual API: implement clipping of obstacle sources
* A* grid reimplemented as unordered_map for maximum efficiency
* smoother obstacle cost function
* Add more profiler and debug traces
* remove -Wabi warnings
* viz: start with camera pointing to the path origin
* Contributors: Jose Luis Blanco-Claraco

0.1.5 (2024-09-16)
------------------
* Update RTTI macros for upcoming MRPT 2.14.0
* Contributors: Jose Luis Blanco-Claraco

0.1.4 (2024-08-29)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

0.1.3 (2024-05-24)
------------------
* Update badges for ROS2 distros
* bump minimum cmake version to 3.5
* Contributors: Jose Luis Blanco-Claraco

0.1.2 (2024-04-25)
------------------
* Fix usage of (new explicit) TPoint3D constructors
* Update build instructions
* Contributors: Jose Luis Blanco-Claraco

0.1.1 (2024-03-19)
------------------
* Fix usage of obsolete mrpt methods
* update ros badges
* Contributors: Jose Luis Blanco-Claraco

0.1.0 (2023-06-14)
------------------
* First release since initial development in May 2019.
* Contributors: Jose Luis Blanco-Claraco, Shravan S Rai
