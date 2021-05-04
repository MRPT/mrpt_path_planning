project(selfdriving-simulator-gui LANGUAGES CXX)

# find dependencies:
find_package(mvsim-simulator QUIET)

if (NOT mvsim-simulator_FOUND)
    message(STATUS "mvsim not found, skipping building the simulator GUI app")
    return()
endif()

find_package(MRPT REQUIRED COMPONENTS tclap)

add_executable(${PROJECT_NAME}
    selfdriving-simulator-gui.cpp
    )

target_link_libraries(${PROJECT_NAME}
    mrpt::tclap
    selfdriving
    mvsim::mvsim-simulator
)

if((NOT MSVC) AND ((NOT CMAKE_CROSSCOMPILING) AND (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")))
  target_compile_options(${PROJECT_NAME} PRIVATE -O3)
endif()
