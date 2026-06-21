project(tests_mrpt_path_planning LANGUAGES CXX)

find_package(GTest QUIET)
if (NOT GTest_FOUND)
    message(WARNING "GTest not found, skipping unit tests.")
    return()
endif()

enable_testing()

selfdriving_add_test(
    TARGET  test_costmap
    SOURCES test_costmap.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_heuristic
    SOURCES test_heuristic.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_astar_holonomic
    SOURCES test_astar_holonomic.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_astar_diffdrive
    SOURCES test_astar_diffdrive.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_astar_diffdrive_bidir
    SOURCES test_astar_diffdrive_bidir.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_viz_svg
    SOURCES test_viz_svg.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_edge_interpolation
    SOURCES test_edge_interpolation.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_obstacle_cache
    SOURCES test_obstacle_cache.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_motion_tree
    SOURCES test_motion_tree.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_cost_evaluator_waypoint
    SOURCES test_cost_evaluator_waypoint.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_best_trajectory
    SOURCES test_best_trajectory.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_refine_trajectory
    SOURCES test_refine_trajectory.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)

selfdriving_add_test(
    TARGET  test_trajectories
    SOURCES test_trajectories.cpp
    LINK_LIBRARIES mrpt_path_planning GTest::GTest GTest::Main
)
