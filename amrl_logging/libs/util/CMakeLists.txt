
add_library(amrl_logging_util src/util.cpp)
target_link_libraries(amrl_logging_util 
  ${catkin_LIBRARIES}
  ${SQLITE3_LIBRARIES})
add_dependencies(amrl_logging_util ${PROJECT_NAME}_generate_messages)
