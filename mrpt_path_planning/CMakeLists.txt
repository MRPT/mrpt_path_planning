project(mrpt_path_planning LANGUAGES CXX)

file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

# define lib:
selfdriving_add_library(
	TARGET
		mrpt_path_planning
	SOURCES
		${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES 
		mrpt::nav mrpt::graphs mrpt::containers mrpt::gui
#	[PRIVATE_LINK_LIBRARIES lib3 lib4]
	CMAKE_DEPENDENCIES 
		mrpt-containers mrpt-graphs mrpt-gui mrpt-nav
)

