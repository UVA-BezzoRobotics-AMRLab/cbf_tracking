

add_executable(threadsafe_queue_test main.cpp)
target_link_libraries(threadsafe_queue_test util ${CMAKE_THREAD_LIBS_INIT})
