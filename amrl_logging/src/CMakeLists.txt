##
# ROS node to recieve and log data to datebase  
##
add_executable(sqlite_node sqlite_node.cpp)
target_link_libraries(sqlite_node
  log_data_handler
  ${catkin_LIBRARIES}
)
add_dependencies(sqlite_node ${PROJECT_NAME}_generate_messages)
