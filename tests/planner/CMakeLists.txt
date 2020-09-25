add_subdirectory("parabolic")
add_subdirectory("ompl")
add_subdirectory("vectorfield")
add_subdirectory("kunzretimer")

aikido_add_test(test_SnapPlanner test_SnapPlanner.cpp)
target_link_libraries(test_SnapPlanner
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner")

aikido_add_test(test_World test_World.cpp)
target_link_libraries(test_World
  "${PROJECT_NAME}_planner")

if(NOT TARGET ${PROJECT_NAME}_planner_dart)
  return()
endif()

aikido_add_test(test_DartPlanners test_DartPlanners.cpp)
target_link_libraries(test_DartPlanners
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner"
  "${PROJECT_NAME}_planner_dart")
