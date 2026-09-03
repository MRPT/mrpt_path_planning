project(selfdriving-simulator-gui LANGUAGES CXX)

if (NOT mvsim-simulator_FOUND OR NOT mvsim-comms_FOUND)
    message(STATUS "mvsim not found, skipping building the simulator GUI app")
    return()
endif()

selfdriving_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    selfdriving-simulator-gui.cpp
    MVSIM_VehicleInterface.h
  LINK_LIBRARIES
    CLI11::CLI11
    mpp::mrpt_path_planning
    mvsim::mvsim-simulator
    mvsim::mvsim-comms
)
