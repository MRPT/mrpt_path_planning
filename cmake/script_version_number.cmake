
# Extract version from package.xml
# Example line:" <version>0.3.2</version>"
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(SELFDRIVING_VERSION ${CMAKE_MATCH_1})
message(STATUS "${PROJECT_NAME} version: ${SELFDRIVING_VERSION} (detected in package.xml)")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${SELFDRIVING_VERSION})
set(SELFDRIVING_MAJOR_VERSION ${CMAKE_MATCH_1})
set(SELFDRIVING_MINOR_VERSION ${CMAKE_MATCH_2})
set(SELFDRIVING_PATCH_VERSION ${CMAKE_MATCH_3})

file(MAKE_DIRECTORY ${SelfdrivingProject_BINARY_DIR}/include/selfdriving/)
configure_file(
	cmake/version.h.in
	${SelfdrivingProject_BINARY_DIR}/include/selfdriving/version.h
	@ONLY
	)
