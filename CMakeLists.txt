cmake_minimum_required(VERSION 2.8.3)
project(traj_generator)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  swarmtal_msgs
)

find_package(Eigen3)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_generator
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(traj_test src/traj_test.cpp src/trajectory_generator_waypoint.cpp)
target_link_libraries(traj_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})
