set(sources
  ConfigurationToConfiguration.cpp
  ConfigurationToConfigurationPlanner.cpp
  ConfigurationToConfigurations.cpp
  ConfigurationToConfigurationsPlanner.cpp
  FirstSupportedMetaPlanner.cpp
  CompositePlanner.cpp
  Planner.cpp
  PlanningResult.cpp
  Problem.cpp
  RankedMetaPlanner.cpp
  SequenceConfigurationToConfigurationMetaPlanner.cpp
  SequenceMetaPlanner.cpp
  SnapConfigurationToConfigurationPlanner.cpp
  SnapPlanner.cpp
  World.cpp
  WorldStateSaver.cpp
  dart/ConfigurationToConfiguration.cpp
  dart/ConfigurationToConfigurationPlanner.cpp
  dart/ConfigurationToConfigurations.cpp
  dart/ConfigurationToConfigurationsPlanner.cpp
  dart/ConfigurationToEndEffectorOffset.cpp
  dart/ConfigurationToEndEffectorPose.cpp
  dart/ConfigurationToTSR.cpp
  dart/ConfigurationToEndEffectorOffsetPlanner.cpp
  # dart/ConfigurationToEndEffectorPosePlanner.cpp
  dart/ConfigurationToTSRPlanner.cpp
  dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.cpp
  dart/ConfigurationToConfiguration_to_ConfigurationToConfigurations.cpp
  dart/ConfigurationToConfiguration_to_ConfigurationToTSR.cpp
  dart/util.cpp
)

add_library("${PROJECT_NAME}_planner" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_planner"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_trajectory"
    ${DART_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories("${PROJECT_NAME}_planner" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)
target_compile_options("${PROJECT_NAME}_planner"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner)
add_component_targets(${PROJECT_NAME} planner "${PROJECT_NAME}_planner")
add_component_dependencies(${PROJECT_NAME} planner
  common
  constraint
  distance
  statespace
  trajectory
)

clang_format_add_sources(${sources})

add_subdirectory("ompl")        # [constraint], [distance], [statespace], [trajectory], dart, ompl
add_subdirectory("parabolic")   # [external], [common], [trajectory], [statespace], dart
add_subdirectory("vectorfield") # [common], [trajectory], [statespace], dart
add_subdirectory("kunzretimer") # [external], [common], [trajectory], [statespace], dart
