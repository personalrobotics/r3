if(CMAKE_COMPILER_IS_GNUCXX)
  if(OMPL_VERSION VERSION_GREATER 1.2.0 OR OMPL_VERSION VERSION_EQUAL 1.2.0)
    if(Boost_VERSION VERSION_LESS 106501)
      message(STATUS "OMPL planners are disabled for OMPL (>=1.2.0) + GCC + "
          "Boost (< 1.65.1). For details, please see: "
          " https://github.com/personalrobotics/aikido/issues/363"
      )
      return()
    endif()
  endif()
endif()

#==============================================================================
# Libraries
#
set(sources
  ConfigurationToConfiguration.cpp
  ConfigurationToConfigurationPlanner.cpp
  ConfigurationToConfiguration_to_ConfigurationToConfiguration.cpp
  ConfigurationToConfiguration_to_ConfigurationToTSR.cpp
  ConfigurationToEndEffectorOffset.cpp
  ConfigurationToEndEffectorPose.cpp
  ConfigurationToEndEffectorOffsetPlanner.cpp
  ConfigurationToTSR.cpp
  ConfigurationToTSRPlanner.cpp
  ConfigurationToTSRwithTrajectoryConstraint.cpp
  CRRTConfigurationToTSRwithTrajectoryConstraintPlanner.cpp
  util.cpp
)

add_library("${PROJECT_NAME}_planner_dart" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_dart" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_dart"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_planner"
    "${PROJECT_NAME}_planner_ompl"
    ${DART_LIBRARIES}
    ${OMPL_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_planner_dart"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_dart)
add_component_targets(${PROJECT_NAME} planner_dart "${PROJECT_NAME}_planner_dart")
add_component_dependencies(${PROJECT_NAME} planner_dart
  constraint
  distance
  planner
  planner_ompl
  statespace
  trajectory
)

clang_format_add_sources(${sources})
