
add_executable(log_write_cpp log_write_cpp.cpp)
target_link_libraries(log_write_cpp
  log_data_handler
  amrl_logging_util
  ${catkin_LIBRARIES}
)
add_dependencies(log_write_cpp ${PROJECT_NAME}_generate_messages)


add_executable(log_write_cpp_2 log_write_cpp_2.cpp)
target_link_libraries(log_write_cpp_2
  log_data_handler
  amrl_logging_util
  ${catkin_LIBRARIES}
)
add_dependencies(log_write_cpp ${PROJECT_NAME}_generate_messages)


add_executable(log_read_cpp log_read_cpp.cpp)
target_link_libraries(log_read_cpp amrl_logging_util)
