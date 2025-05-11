cmake_minimum_required(VERSION 3.0.2)
project(cbf_tracking VERSION 1.0 LANGUAGES CXX)

# Ensure C++17 is used
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# compilation options
add_compile_options(-O3)

# Use custom findCMAKE files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  costmap_2d
  sensor_msgs
  amrl_logging
  grid_map_ros
  geometry_msgs
  trajectory_msgs
  distance_map_core
  crazyswarm
)

# Using GUROBI_LIBRARY on it's own isn't enough to get things to compile...
# Snippet taken from https://github.com/mit-acl/faster/blob/master/faster/CMakeLists.txt
FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};" )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(GUROBI REQUIRED)
find_package(GiNaC REQUIRED)
find_package(nlopt REQUIRED)
# find_package(CGAL REQUIRED)

find_package(pybind11 QUIET)

add_message_files(
      DIRECTORY
      FILES
      SolverState.msg
)

add_service_files(
  FILES
  SolveFromState.srv
  QuerySAC.srv
  GetState.srv
)

generate_messages(
      DEPENDENCIES 
      nav_msgs
      geometry_msgs 
      trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  third_party/autodiff
  ${catkin_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
  ${GINAC_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    ## Find gtest
    find_package(GTest REQUIRED)

    add_executable(test_filter_nlopt
        src/mpc/mpc_nlopt.cpp
        src/filter/filter_nlopt.cpp
        tests/test_filter_nlopt.cpp
    )
    target_link_libraries(test_filter_nlopt
        ${catkin_LIBRARIES}
        ${NLOPT_LIBRARIES}
        Eigen3::Eigen
        GTest::GTest
    )

    enable_testing()

    add_test(NAME test_filter_nlopt COMMAND test_filter_nlopt)
endif()

# add_library(${PROJECT_NAME}_rviz_panel src/rviz_panel.cpp)

# target_link_libraries(${PROJECT_NAME}_rviz_panel
#   ${catkin_LIBRARIES}
# )

# install(TARGETS ${PROJECT_NAME}_rviz_panel
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(FILES plugin_description.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mpc_trajectory_tracking_node.cpp)
if(pybind11_FOUND)
    message(STATUS "Found pybind11, building mpc python bindings")
    add_definitions(-DFOUND_PYBIND11)

    # find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    find_package(PythonLibs REQUIRED)
    include_directories(${PYTHON_INCLUDE_DIRS})

    pybind11_add_module(py_mpc 
        src/mpc/py_mpc.cpp 
        src/mpc/mpc_core.cpp
        src/mpc/mpc_nlopt.cpp
        src/mpc/jackal_mpc_pos.cpp
        src/mpc/jackal_mpc_acceleration.cpp
        src/mpc/jackal_mpc_trajectory_tracking.cpp
        
        src/filter/filter_nlopt.cpp
        src/filter/filter_nlopt_horizon.cpp
    )
    target_link_libraries(py_mpc
        PRIVATE ${distance_map_core_LIBRARIES}
        PRIVATE ${NLOPT_LIBRARIES}
        PRIVATE Eigen3::Eigen
        PRIVATE ipopt
    )

    pybind11_add_module(py_filter
        src/filter/py_filter.cpp
        src/filter/filter_nlopt.cpp
    )
    target_link_libraries(py_filter
        PRIVATE ${distance_map_core_LIBRARIES}
        PRIVATE ${NLOPT_LIBRARIES}
        PRIVATE Eigen3::Eigen
    )

else()
    message(STATUS "Did not find pybind11, not building mpc python bindings")
endif()

add_executable(jackal_mpc_tracking
            src/mpc/mpc_core.cpp
            src/mpc/mpc_nlopt.cpp
            src/mpc/jackal_mpc_pos.cpp
            src/mpc/jackal_mpc_acceleration.cpp
            src/mpc/jackal_mpc_trajectory_tracking.cpp
            src/mpc/jackal_mpc_ros_trajectory_tracking.cpp
            src/mpc/jackal_mpc_trajectory_tracking_node.cpp

            src/filter/filter_nlopt.cpp
            src/filter/filter_nlopt_horizon.cpp)
target_link_libraries(jackal_mpc_tracking
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
    Eigen3::Eigen
    ipopt
)
add_dependencies(jackal_mpc_tracking ${PROJECT_NAME}_generate_messages)

add_executable(test_mpc_solver 
    src/utils/test_mpc_solver.cpp
    src/filter/filter_nlopt_horizon.cpp
)
target_link_libraries(test_mpc_solver
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
    Eigen3::Eigen
)

add_executable(jackal_unicycle_mpc 
    src/sim/jackal_unicycle_mpc.cpp
    src/filter/safety_filter_unicycle.cpp
)
target_link_libraries(jackal_unicycle_mpc
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ipopt
)

add_executable(jackal_extended_unicycle_mpc
src/sim/jackal_extended_unicycle_mpc.cpp
src/filter/safety_filter_extended_unicycle.cpp
)
target_link_libraries(jackal_extended_unicycle_mpc
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ipopt
)

add_executable(unicycle_sim src/sim/unicycle_sim.cpp)
target_link_libraries(unicycle_sim
    ${catkin_LIBRARIES}
    Eigen3::Eigen
)
add_dependencies(jackal_mpc_tracking ${PROJECT_NAME}_generate_messages)

add_executable(crazyflie_sim src/sim/crazyflie_sim.cpp)
target_link_libraries(crazyflie_sim
    ${catkin_LIBRARIES}
    Eigen3::Eigen
)

add_executable(jackal_extended_unicycle_test
    src/sim/jackal_extended_unicycle_test.cpp
    src/filter/safety_filter_extended_unicycle.cpp
)
target_link_libraries(jackal_extended_unicycle_test
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ipopt
)

add_executable(planner
    src/planner/JPS.cpp
    src/planner/solver.cpp 
    src/planner/planner_core.cpp
    src/planner/planner_gurobi.cpp 
    src/planner/planner_gurobi_node.cpp 
)
target_link_libraries(planner
    ${catkin_LIBRARIES}
    ${GINAC_LIBRARIES}
    ${GUROBI_LIBRARIES}
    ${NLOPT_LIBRARIES}
    Eigen3::Eigen
    # CGAL::CGAL
    ipopt
)

# add_executable(generate_traj_through_map
#     src/planner/JPS.cpp
#     src/planner/solver.cpp 
#     src/planner/generate_traj_through_map.cpp
# )
# target_link_libraries(generate_traj_through_map
#     ${catkin_LIBRARIES}
#     ${GINAC_LIBRARIES}
#     ${GUROBI_LIBRARIES}
#     ${NLOPT_LIBRARIES}
#     Eigen3::Eigen
# )

add_executable(local_costmap src/utils/local_costmap.cpp)
target_link_libraries(local_costmap
    ${catkin_LIBRARIES}
)

add_executable(solver_server 
    src/utils/solve_from_state_server.cpp
    src/planner/solver.cpp
    src/planner/JPS.cpp
)
target_link_libraries(solver_server
  ${catkin_LIBRARIES}
  ${GUROBI_LIBRARIES}
  ${GINAC_LIBRARIES}
  Eigen3::Eigen
)
add_dependencies(solver_server ${PROJECT_NAME}_generate_messages)

add_executable(publish_pf_pose src/utils/publish_pf_pose.cpp)
target_link_libraries(publish_pf_pose
  ${catkin_LIBRARIES}
)

add_executable(lidar_simulator src/sim/lidar_simulator.cpp)
target_link_libraries(lidar_simulator
  ${catkin_LIBRARIES}
)

add_executable(test_reparam src/utils/test.cpp)
target_link_libraries(test_reparam
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
