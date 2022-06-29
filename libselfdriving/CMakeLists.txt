project(selfdriving LANGUAGES CXX)

file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

# find dependencies:
find_package(MRPT 2.3.2 REQUIRED COMPONENTS containers graphs gui nav)

# define lib:
selfdriving_add_library(
  TARGET selfdriving
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES ${MRPT_LIBRARIES}
#	[PRIVATE_LINK_LIBRARIES lib3 lib4]
	CMAKE_DEPENDENCIES mrpt-containers mrpt-graphs mrpt-gui mrpt-nav
)

