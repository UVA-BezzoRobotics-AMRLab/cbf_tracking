cmake_minimum_required(VERSION 3.0.2)
project(amrl_logging)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:-std=c17>
  $<$<COMPILE_LANGUAGE:CXX>:-lsqlite3>
  -pthread
  -O3
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  std_msgs
  message_generation
)
find_package(Threads) # c++ threads library
find_package(SQLite3) # SQLite3 library
find_package(Boost)   # Boost software libraries

## Specify additional locations of header files
include_directories(${PROJECT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(DIRECTORY ${PROJECT_SOURCE_DIR}/ros_msg
  FILES
  LoggingData.msg
)

add_service_files(DIRECTORY ${PROJECT_SOURCE_DIR}/ros_srv
  FILES
  LoggingStart.srv
  LoggingDropTable.srv
  LoggingBufferCheck.srv
  LoggingStop.srv
)

roslaunch_add_file_check(ros_launch)

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  LIBRARIES amrl_logging_util
  INCLUDE_DIRS include
)

####################
## Subdirectories ##
####################

add_subdirectory(libs)
add_subdirectory(src)
add_subdirectory(examples)
