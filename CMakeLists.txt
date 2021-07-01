# CMake 2.8.12 or above is required for CMakeParseArguments.
# CMake 2.8.11 or above is required for target_include_directories.
cmake_minimum_required(VERSION 2.8.12)

# Set the CMAKE_CXX_COMPILER_ID variable to AppleClang instead of Clang.
# AppleClang and Clang have different version number. This was introduced in
# CMake 3.0.
if(POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif()

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

project(aikido)

# If you change the version, please update the <version> tag in package.xml.
set(AIKIDO_MAJOR_VERSION "0")
set(AIKIDO_MINOR_VERSION "4")
set(AIKIDO_PATCH_VERSION "0")
set(AIKIDO_VERSION "${AIKIDO_MAJOR_VERSION}.${AIKIDO_MINOR_VERSION}.${AIKIDO_PATCH_VERSION}")

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

#==============================================================================
# Configuration Options
#
set(INCLUDE_INSTALL_DIR "include")
set(LIBRARY_INSTALL_DIR "lib")
set(CONFIG_INSTALL_DIR "${LIBRARY_INSTALL_DIR}/${PROJECT_NAME}/cmake")

option(CODECOV "Enable codecov support" OFF)
option(DOWNLOAD_TAGFILES "Download Doxygen tagfiles for dependencies" OFF)
option(TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(BUILD_AIKIDOPY "Build aikidopy (the python binding)" OFF)

if(BUILD_AIKIDOPY)
  set(BUILD_SHARED_LIBS OFF)
endif()

#==============================================================================
# codecov Setup
#
if(CODECOV)
  include(CodeCoverage)
  setup_target_for_coverage(${PROJECT_NAME}_coverage ctest coverage)
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage"
  )
endif()

#==============================================================================
# Helper functions.
#
function(aikido_space_delimit output_variable input_list)
  set(accumulator)
  foreach(input_element ${input_list})
    set(accumulator "${accumulator} ${input_element}")
  endforeach()

  set("${output_variable}" "${accumulator}" PARENT_SCOPE)
endfunction()

macro(aikido_check_package variable component dependency)
  if(${${variable}_FOUND})
    message(STATUS "Looking for ${dependency} - version ${${variable}_VERSION}"
                   " found")
  else()
    message(WARNING "Looking for ${dependency} - NOT found, to use"
                    " ${component}, please install ${dependency}")
    return()
  endif()
endmacro()

#==============================================================================
# Required Dependencies
#
include(Components)
initialize_component_helpers(aikido)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(ClangFormat)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(DART 6.6.2 REQUIRED
  COMPONENTS optimizer-nlopt utils
  OPTIONAL_COMPONENTS utils-urdf # for 'perception' target
  CONFIG
)

find_package(ompl REQUIRED)

#==============================================================================
# Compiler settings
#
# CompilerSettings optionally requires a custon CMake option
# TREAT_WARNINGS_AS_ERRORS.
include(CompilerSettings)

#==============================================================================
# Libraries and unit tests.
#
configure_file(
  ${CMAKE_MODULE_PATH}/version.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/aikido/version.hpp
  @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/aikido/version.hpp
  DESTINATION "${INCLUDE_INSTALL_DIR}/aikido")

include_directories("include")

# This custom target doesn't affect on building aikido projects but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE aikido_headers "include/*.hpp")
add_custom_target(headers SOURCES ${aikido_headers})

add_subdirectory("src")

if(BUILD_AIKIDOPY)
  add_subdirectory("python")
endif()

enable_testing()
add_subdirectory("tests" EXCLUDE_FROM_ALL)

# Targets to mimic a Catkin package. "tests" builds tests and "test" (or
# "run_tests") runs them.
get_property(all_tests GLOBAL PROPERTY AIKIDO_TESTS)
add_custom_target(tests DEPENDS ${all_tests})
add_custom_target(run_tests COMMAND "${CMAKE_CTEST_COMMAND}")

#==============================================================================
# Doxygen.
#
find_package(Doxygen QUIET)

if(${DOXYGEN_FOUND})
  message(STATUS "Looking for Doxygen - version ${DOXYGEN_VERSION} found")

  set(DOXYGEN_INPUT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set(DOXYGEN_OUTPUT_ROOT "${CMAKE_CURRENT_BINARY_DIR}/doxygen")
  set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/aikido.tag")
  set(DOXYGEN_HTML_INDEX "${DOXYGEN_OUTPUT_ROOT}/index.html")
  set(DOXYGEN_WORKING_DIR "${CMAKE_CURRENT_BINARY_DIR}/doxygen_working")
  set(DOXYGEN_EXCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/external ${CMAKE_CURRENT_SOURCE_DIR}/src/python")
  set(DOXYGEN_STRIP_FROM_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

  # Strip include_directories() being used by CMake from #include paths.
  # TODO: Add per-target INCLUDE_DIRECTORIES properties to this list.
  get_property(DOXYGEN_STRIP_FROM_INC_PATH_LIST
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    PROPERTY INCLUDE_DIRECTORIES)
  aikido_space_delimit(DOXYGEN_STRIP_FROM_INC_PATH
    "${DOXYGEN_STRIP_FROM_INC_PATH_LIST}")

  # Optionally download Doxygen tagfiles for dependencies.
  set(DOXYGEN_TAGFILES "")
  if(DOWNLOAD_TAGFILES)
    set(tagfile_url "http://dartsim.github.io/dart/v6.3.0/dart.tag")
    message(STATUS "Downloading DART Doxygen tagfile from: ${tagfile_url}")
    file(DOWNLOAD "${tagfile_url}" "${DOXYGEN_WORKING_DIR}/dart.tag")
    set(DOXYGEN_TAGFILES
      "${DOXYGEN_TAGFILES} dart.tag=http://dartsim.github.io/dart/v6.3.0")

    is_component(needs_ompl aikido planner_ompl)
    if(needs_ompl)
      set(tagfile_url "http://ompl.kavrakilab.org/core/ompl.tag")
      message(STATUS "Downloading OMPL Doxygen tagfile from: ${tagfile_url}")
      file(DOWNLOAD "${tagfile_url}" "${DOXYGEN_WORKING_DIR}/ompl.tag")
      set(DOXYGEN_TAGFILES
        "${DOXYGEN_TAGFILES} ompl.tag=http://ompl.kavrakilab.org/")
    endif()
  endif()

  # Generate a Doxyfile. This uses the variables:
  #
  # - DOXYGEN_EXCLUDE
  # - DOXYGEN_EXTRA_INPUTS
  # - DOXYGEN_GENERATE_TAGFILE
  # - DOXYGEN_INPUT_ROOT
  # - DOXYGEN_OUTPUT_ROOT
  # - DOXYGEN_STRIP_FROM_PATH
  # - DOXYGEN_STRIP_FROM_INC_PATH
  # - DOXYGEN_TAGFILES
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in"
    "${DOXYGEN_WORKING_DIR}/Doxyfile" @ONLY)

  # Create the "docs" target that runs Doxygen.
  add_custom_command(OUTPUT "${DOXYGEN_HTML_INDEX}"
    COMMAND "${DOXYGEN_EXECUTABLE}"

    # Fix missing ^ in generated navtree.js for external DART link
    COMMAND sed -i s:\"http:\"\^http:g ${DOXYGEN_OUTPUT_ROOT}/navtree.js

    # Strip path prefix from all paths in aikido.tag
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping paths from"
        "${DOXYGEN_GENERATE_TAGFILE}"
    COMMAND sed -i s:${DOXYGEN_STRIP_FROM_PATH}::g ${DOXYGEN_GENERATE_TAGFILE}

    # Strip all doxygen="path" HTML tags
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping Doxygen HTML tags"
    COMMAND find "${DOXYGEN_OUTPUT_ROOT}" -type f -name "*.html"
        -exec sed -i 's: doxygen=\"[^\"]*\"::g' {} \\$<SEMICOLON>

    DEPENDS "${DOXYGEN_WORKING_DIR}/Doxyfile"
    WORKING_DIRECTORY "${DOXYGEN_WORKING_DIR}"
  )

  add_custom_target(docs
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Generating documentation."
  )
  add_custom_target(
    docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Generating documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    WORKING_DIRECTORY "${DOXYGEN_WORKING_DIR}"
  )

  # Create the "view_docs" target that opens the documentation.
  if(APPLE)
    set(open_command "open")
  else()
    set(open_command "xdg-open")
  endif()

  add_custom_target(view_docs "${open_command}" "${DOXYGEN_HTML_INDEX}"
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Opening documentation in a web browser.")
else()
  message(STATUS "Looking for Doxygen - NOT found, to generate API"
                 " documentation, please install Doxygen")
endif()

#===============================================================================
# Automatic code formatting using clang-format.
#
# We only support one specific ClangFormat version because different versions
# result in different code formatting with the same configuration. The current
# supported version is 6.0.

clang_format_setup(VERSION 6.0)

if (CLANG_FORMAT_EXECUTABLE)
  clang_format_add_sources(${aikido_headers})

  clang_format_add_targets()
endif()

#==============================================================================
# Installation.
#
install(DIRECTORY "include/"
  DESTINATION "${INCLUDE_INSTALL_DIR}")

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/${PROJECT_NAME}")

# Generate and install CMake configuration files for each component <C>:
# - <C>Component.cmake, which defines:
#   - aikido_<C>_DEPENDENCIES: list of component dependencies
#   - aikido_<C>_LIBRARIES: list of library targets in this component
# - <C>Targets.cmake, which creates IMPORTED targets
install_component_exports(aikido)

# Generate and install a Config.cmake file. This file includes the
# <C>Component.cmake and <C>Targets.cmake created above. It also uses the
# following variables:
#
# - PACKAGE_INCLUDE_INSTALL_DIR
# - PACKAGE_INCLUDE_DIRS
get_property(PACKAGE_INCLUDE_DIRS GLOBAL
  PROPERTY "${PROJECT_NAME}_INCLUDE_DIRS")
configure_package_config_file("cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS INCLUDE_INSTALL_DIR)
write_basic_config_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${AIKIDO_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "${CONFIG_INSTALL_DIR}")

#==============================================================================
# Uninstallation.
#
# Add a custom target "uninstall"
# Ref: http://www.cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
  "${PROJECT_BINARY_DIR}/uninstall_target.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

#==============================================================================
# Check if DART is built with octomap so that it supports VoxelGridShape
# Do this here to avoid mutating CMAKE variables.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_DEFINITIONS "")
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
set(CMAKE_REQUIRED_INCLUDES ${DART_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${DART_LIBRARIES})
check_cxx_source_compiles(
"
#include <dart/dart.hpp>
int main()
{
  auto voxel = new dart::dynamics::VoxelGridShape();
  delete voxel;
  return 0;
}
"
DART_HAS_VOXELGRIDSHAPE
)