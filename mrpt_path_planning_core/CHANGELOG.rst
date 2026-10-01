^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_path_planning_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-10-01)
------------------
* add missing changelogs
* Merge pull request `#44 <https://github.com/MRPT/mrpt_path_planning/issues/44>`_ from MRPT/feat/point-goal-shot
* TPS_Astar: commit a pending Reeds-Shepp shot if the search stops before its commit test
* Point-goal shots: other final headings and no shots where the robot cannot fit
* Reeds-Shepp shot to position goals
* Merge pull request `#43 <https://github.com/MRPT/mrpt_path_planning/issues/43>`_ from MRPT/docs/readme-agents-md
* README: animated header images of the demo plans
* Refined path outputs show the motion actually executed
* viz_svg: optional animated robot footprint along the path
* Merge pull request `#41 <https://github.com/MRPT/mrpt_path_planning/issues/41>`_ from MRPT/feat/mrpt-batched-tp-obstacles-compat
* Mark updateTPObstacles() override with MRPT >= 3.3.2
* Merge pull request `#40 <https://github.com/MRPT/mrpt_path_planning/issues/40>`_ from MRPT/feat/fast-collision-query
* Interpolate Reeds-Shepp shot edges and keep exact edges when refining
* Enable the Reeds-Shepp shot by default
* Certified Reeds-Shepp shot to full-pose goals (opt-in)
* Faster batched TP-obstacle query and open set in TPS_Astar
* Merge pull request `#39 <https://github.com/MRPT/mrpt_path_planning/issues/39>`_ from MRPT/feat/reeds-shepp-heuristic
* reeds_shepp: portable pi constant; document the radius precondition
* Add an opt-in Reeds-Shepp heuristic for full-pose goals
* Merge pull request `#38 <https://github.com/MRPT/mrpt_path_planning/issues/38>`_ from MRPT/ci/clang-format
* Add clang-format CI check and apply clang-format-14 to all sources
* Merge pull request `#37 <https://github.com/MRPT/mrpt_path_planning/issues/37>`_ from MRPT/feat/certified-collision-grid
* Address PR review comments: bidir expanded-node count, final-step reach margin, one-edge path check
* Collision grid: O(1) inside-footprint rejection in obstacle queries, faster build
* Certified collision grid, footprint-aware obstacle clipping, consistent tree poses
* Merge pull request `#36 <https://github.com/MRPT/mrpt_path_planning/issues/36>`_ from MRPT/feat/mrpt3
* Address PR review comments and finish the getPathStepForDist port
* Port to MRPT 3.x: re-port after the core/apps/viz repo split
* TrajectoryFollower: latch ReachedGoal until a new trajectory is set
* TrajectoryFollower: parameterize the control-pose filter at the vehicle
* TrajectoryFollower: noise-robustness and accuracy hardening
* More info in debug traces
* add debug-level traces
* more stable look ahead, cap max omega
* TrajectoryFollower: evaluate off-path fault on true localized pose
* TrajectoryFollower: run short-term tracking on wheel odometry via a slewed anchor
* TrajectoryFollower: curvature-adaptive lookahead (replaces velocity schedule)
* TrajectoryFollower: gate goal/arrival on arc-length, not Euclidean distance only
* TrajectoryFollower: fix backward-propagated gear misclassifying a
* TrajectoryFollower: support genuine mid-path gear reversals
* follower: clamp curvature to min_turn_radius, decelerate before cusps too
* follower: fix short cusped-reverse hang (lookahead across cusp + arrival latch)
* Fix: viz.h must stay a properly exported public library, not a private CLI header
* Split into mrpt_path_planning_core / _apps / metapackage; flatten follow/
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

1.0.1 (2026-07-04)
------------------
* feat: bidirectional TP-Space A* solver (TPS_Astar_Bidir) (`#32 <https://github.com/MRPT/mrpt_path_planning/issues/32>`_)
* feat(cli): add --no-gui and SVG motion-tree decimation (`#31 <https://github.com/MRPT/mrpt_path_planning/issues/31>`_)
* fix(TPS_Astar): defer analytic goal expansion to avoid suboptimal goal loops (`#30 <https://github.com/MRPT/mrpt_path_planning/issues/30>`_)
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2026-06-16)
------------------
* feat(TPS_Astar): weighted A* via ``heuristic_epsilon`` param (default 1.0 = exact A*); eps in [1.5, 2] cuts median plan time ~4-5x at ~3-8% longer paths
* feat(TPS_Astar): analytic expansion / early termination to goal (`#25 <https://github.com/MRPT/mrpt_path_planning/issues/25>`_): accept a direct-to-goal PTG edge as soon as it lands in the goal cell, skipping unnecessary expansions
* feat(TPS_Astar): optional 2D-Dijkstra obstacle-aware heuristic (`#24 <https://github.com/MRPT/mrpt_path_planning/issues/24>`_): opt-in ``use_obstacle_heuristic`` pre-computes a cost-to-go grid; ~5x fewer expansions on hard BARN worlds
* feat(TPS_Astar): wire ``stateGoal.vel`` into goal-cell exit speed (`#18 <https://github.com/MRPT/mrpt_path_planning/issues/18>`_)
* feat(viz): SVG exporter for plans (`#29 <https://github.com/MRPT/mrpt_path_planning/issues/29>`_): ``mpp::save_plan_to_svg()`` — dependency-free 2D vector export of motion tree, solution path, footprint, and start/goal markers; new ``--save-svg`` CLI option
* feat(gui): viz now shows selected point coordinates
* perf(TPS_Astar): batch TP-obstacle collision check per node expansion (`#26 <https://github.com/MRPT/mrpt_path_planning/issues/26>`_): O(candidates×N) → O(N); ~2-3x faster on hard BARN worlds
* perf(TPS_Astar): defer per-edge path interpolation to the final solution (`#27 <https://github.com/MRPT/mrpt_path_planning/issues/27>`_): ~17-21% additional speedup
* perf(TPS_Astar): coarsen default ``grid_resolution_yaw`` from 5° to 7.5°; ~23-37% faster with equal or better success rate on BARN/HouseExpo benchmarks
* perf(P3): xy-cell obstacle cache in ``cached_local_obstacles()`` (`#20 <https://github.com/MRPT/mrpt_path_planning/issues/20>`_): ~6x per-call speedup; cache keyed by (ix,iy) and cleared each ``plan()``
* fix(TPS_Astar): obstacle cache ignored heading, causing collision false negatives (`#28 <https://github.com/MRPT/mrpt_path_planning/issues/28>`_): cache now stores heading-independent clipped cloud; heading-dependent transform done per call
* fix: ``refine_trajectory()`` distance overshoot
* fix: SVG draw now honors any-heading goals
* fix: restore A* admissibility by converting heuristic cost to seconds (`#16 <https://github.com/MRPT/mrpt_path_planning/issues/16>`_)
* fix: store ``ptgTrimmableSpeed`` in best-path replacement block (`#15 <https://github.com/MRPT/mrpt_path_planning/issues/15>`_)
* fix: replace ``NodeCoordsHash`` with ``boost::hash_combine`` avalanche pattern to eliminate bucket clustering in large lattices
* fix: bind ``obs_xs``/``obs_ys`` by const ref in ``transform_pc_square_clipping`` to avoid O(N) copy (`#14 <https://github.com/MRPT/mrpt_path_planning/issues/14>`_)
* fix: remove dead ``THROW_EXCEPTION`` branches in ``edge_interpolated_path`` (`#17 <https://github.com/MRPT/mrpt_path_planning/issues/17>`_)
* test: end-to-end A* tests for C-PTG forward and reverse motion (`#23 <https://github.com/MRPT/mrpt_path_planning/issues/23>`_)
* test: add UnreachableGoal, Timeout, MultiPTG, MotionTree, and obstacle-cache unit tests (`#22 <https://github.com/MRPT/mrpt_path_planning/issues/22>`_)
* build: bump minimum CMake version to 3.22 (`#19 <https://github.com/MRPT/mrpt_path_planning/issues/19>`_)
* ci: install ``libgtest-dev`` so GTest suite is actually built and run
* docs: add ROS 2 Lyrical badge, update Rolling to Ubuntu 26.04 (Resolute)
* Contributors: Jose Luis Blanco-Claraco, SRai22

0.3.0 (2026-04-02)
------------------
* copyright year bump
* Integrate vscode config to build with colcon
* Add basic unit tests
* add params for ackermann
* Fix potential wrong lattice coordinates due to bad rounding
* Fix: use smooth quadratic cost
* Fix potential phi angle comparison wraps
* Docs: explain optimality criterion
* tune params and add more examples in readme
* fix potential non-optimal path search
* fix build against modern mvsim
* Add ARQUITECTURE.md
* Contributors: Jose Luis Blanco-Claraco

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
