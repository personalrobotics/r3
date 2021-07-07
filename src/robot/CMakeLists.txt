#==============================================================================
# Dependencies
#
if(NOT TARGET ${PROJECT_NAME}_planner_ompl)
  return()
endif()
set(CMAKE_REQUIRED_DEFINITIONS "")
set(CMAKE_REQUIRED_FLAGS "")

#==============================================================================
# Libraries
#
set(sources
  GrabMetadata.cpp
  Robot.cpp
  util.cpp
)

add_library("${PROJECT_NAME}_robot" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_robot" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_robot"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_io"
    "${PROJECT_NAME}_planner"
    "${PROJECT_NAME}_planner_ompl"
    "${PROJECT_NAME}_planner_parabolic"
    "${PROJECT_NAME}_planner_kunzretimer"
    "${PROJECT_NAME}_planner_vectorfield"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    ${DART_LIBRARIES}
)

target_compile_options("${PROJECT_NAME}_robot"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} robot)
add_component_targets(${PROJECT_NAME} robot "${PROJECT_NAME}_robot")
add_component_dependencies(${PROJECT_NAME} robot
  common
  io
  planner
  planner_ompl
  planner_parabolic
  planner_vectorfield
  constraint
  distance
  statespace
  trajectory
)

clang_format_add_sources(${sources})
