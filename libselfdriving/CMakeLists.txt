project(selfdriving LANGUAGES CXX)

# Project version:
# (TODO: Read from file?)
set(SELFDRIVING_VERSION_NUMBER_MAJOR 0)
set(SELFDRIVING_VERSION_NUMBER_MINOR 1)
set(SELFDRIVING_VERSION_NUMBER_PATCH 0)

# We want libraries to be named "libXXX" in all compilers, "libXXX-dbg" in MSVC
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
set(CMAKE_IMPORT_LIBRARY_PREFIX "lib")
set(CMAKE_STATIC_LIBRARY_PREFIX "lib")
set(CMAKE_DEBUG_POSTFIX "-dbg")


# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

add_library(${PROJECT_NAME} SHARED
    ${LIB_SRCS}
    ${LIB_PUBLIC_HDRS}
    )

# find dependencies:
find_package(MRPT 2.0.0 REQUIRED COMPONENTS gui nav)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${MRPT_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION "${SELFDRIVING_VERSION_NUMBER_MAJOR}.${SELFDRIVING_VERSION_NUMBER_MINOR}.${SELFDRIVING_VERSION_NUMBER_PATCH}"
  SOVERSION ${SELFDRIVING_VERSION_NUMBER_MAJOR}.${SELFDRIVING_VERSION_NUMBER_MINOR}
  )


if((NOT MSVC) AND ((NOT CMAKE_CROSSCOMPILING) AND (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")))
  target_compile_options(${PROJECT_NAME} PRIVATE -O3 -mtune=native)
endif()


