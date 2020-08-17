project(plan-path LANGUAGES CXX)

# find dependencies:
find_package(MRPT REQUIRED COMPONENTS gui nav tclap)

add_executable(${PROJECT_NAME}
    plan-path.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${MRPT_LIBRARIES}
	selfdriving
)

if((NOT MSVC) AND ((NOT CMAKE_CROSSCOMPILING) AND (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")))
  target_compile_options(${PROJECT_NAME} PRIVATE -O3 -mtune=native)
endif()
