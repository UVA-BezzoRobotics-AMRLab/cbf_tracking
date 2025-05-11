
add_executable( sql_test sql_table_test.cpp)
target_link_libraries( sql_test sql_table ${CMAKE_THREAD_LIBS_INIT})

add_executable( sql_db_test sql_db_test.cpp)
target_link_libraries( sql_db_test sql_table)
