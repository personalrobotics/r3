if(NOT TARGET ${PROJECT_NAME}_planner_dart)
  return()
endif()

set(sources
  VectorFieldPlanner.cpp
  VectorField.cpp
  VectorFieldUtil.cpp
  VectorFieldConfigurationToEndEffectorOffsetPlanner.cpp
  detail/VectorFieldIntegrator.cpp
  detail/VectorFieldPlannerExceptions.cpp
  BodyNodePoseVectorField.cpp
  MoveEndEffectorOffsetVectorField.cpp
  MoveEndEffectorPoseVectorField.cpp
)

add_library("${PROJECT_NAME}_planner_vectorfield" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_planner_vectorfield"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_planner"
    "${PROJECT_NAME}_planner_dart"
    ${DART_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories("${PROJECT_NAME}_planner_vectorfield" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)
target_compile_options("${PROJECT_NAME}_planner_vectorfield"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_vectorfield)
add_component_targets(${PROJECT_NAME} planner_vectorfield "${PROJECT_NAME}_planner_vectorfield")
add_component_dependencies(${PROJECT_NAME} planner_vectorfield
  common
  constraint
  distance
  statespace
  trajectory
  planner
)

clang_format_add_sources(${sources})
