cmake_minimum_required(VERSION 3.1)

project(SelfdrivingProject LANGUAGES CXX)


# Include std cmake scripts:
include(CheckCXXCompilerFlag) # for check_cxx_compiler_flag()
include(CMakeDetermineCXXCompiler)

# Our custom scripts:
include(cmake/script_version_number.cmake REQUIRED)	#  Loads version number from package.xml
include(cmake/misc_cmake_functions.cmake REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


add_subdirectory(libselfdriving)
add_subdirectory(apps)
