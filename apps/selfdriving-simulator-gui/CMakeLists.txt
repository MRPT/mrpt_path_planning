project(selfdriving-simulator-gui LANGUAGES CXX)

if (NOT mvsim-simulator_FOUND)
    message(STATUS "mvsim not found, skipping building the simulator GUI app")
    return()
endif()

selfdriving_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES selfdriving-simulator-gui.cpp
  LINK_LIBRARIES 
    mrpt::tclap
    mrpt_path_planning
    mvsim::mvsim-simulator
)
