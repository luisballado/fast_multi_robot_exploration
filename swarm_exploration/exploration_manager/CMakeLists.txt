cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  lkh_mtsp_solver
  plan_manage
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  # SentGoals.msg
  # DeletedGoals.msg
  DroneState.msg
  PairOpt.msg
  PairOptResponse.msg
  HGrid.msg
  GridTour.msg
  SearchObstacle.msg
)

generate_messages(
  DEPENDENCIES
  # std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils lkh_tsp_solver lkh_mtsp_solver plan_manage message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# RACER nodes
add_executable(exploration_node
  src/racer/exploration_node.cpp 
  src/racer/fast_exploration_fsm.cpp
  src/racer/fast_exploration_manager.cpp
  )
target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  -ldw
  )

# FAME node
add_executable(fame_node
  src/fame/fame_node.cpp
  src/fame/fame_exploration_fsm.cpp
  src/fame/fame_exploration_manager.cpp
  src/fame/role_assigner.cpp
  src/fame/collaboration_assigner.cpp
)

target_link_libraries(fame_node 
  ${catkin_LIBRARIES}
  -ldw
  )

# MVANT node
add_executable(mvant_node
  src/mvant/mvant_node.cpp
  src/mvant/mvant_exploration_fsm.cpp
  src/mvant/mvant_exploration_manager.cpp
  src/mvant/hungarian.cpp
)

target_link_libraries(mvant_node 
  ${catkin_LIBRARIES}
  -ldw
  )

# Tests
catkin_add_gtest(test_collaboration_cost
    test/collaboration_test.cpp
    src/mvant/collaboration_assigner.cpp
)
target_link_libraries(test_collaboration_cost ${catkin_LIBRARIES})
