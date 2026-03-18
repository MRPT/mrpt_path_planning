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
