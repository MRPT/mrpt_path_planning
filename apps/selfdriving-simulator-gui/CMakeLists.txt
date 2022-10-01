project(selfdriving-simulator-gui LANGUAGES CXX)

# find dependencies:
find_package(mvsim-simulator QUIET)
set(mvsim-simulator_FOUND ${mvsim-simulator_FOUND} PARENT_SCOPE) # to show it in root cmake final summary
set(mvsim-simulator_VERSION ${mvsim-simulator_VERSION} PARENT_SCOPE) # to show it in root cmake final summary

if (NOT mvsim-simulator_FOUND)
    message(STATUS "mvsim not found, skipping building the simulator GUI app")
    return()
endif()

find_package(MRPT REQUIRED COMPONENTS tclap)

selfdriving_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES selfdriving-simulator-gui.cpp
  LINK_LIBRARIES 
    mrpt::tclap
    selfdriving
    mvsim::mvsim-simulator
)
