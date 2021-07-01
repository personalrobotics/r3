# GTest setup
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory("gtest")

# Define aikido_add_test for registering Aikido tests.
set_property(GLOBAL PROPERTY AIKIDO_TESTS)
function(aikido_add_test target_name)
  add_executable("${target_name}" ${ARGN})
  add_test("${target_name}" "${target_name}")

  target_link_libraries("${target_name}" gtest gtest_main)

  set_property(GLOBAL APPEND PROPERTY AIKIDO_TESTS "${target_name}")
  clang_format_add_sources(${ARGN})
endfunction()

# Add helper scripts to the include path.
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory("common")
add_subdirectory("io")
add_subdirectory("constraint")
add_subdirectory("control")
add_subdirectory("distance")
add_subdirectory("planner")
add_subdirectory("statespace")
add_subdirectory("trajectory")

clang_format_add_sources(eigen_tests.hpp)
