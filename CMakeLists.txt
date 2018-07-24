cmake_minimum_required(VERSION 2.8.3)
project(motion_planning)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
  )

add_executable(motion_planning_0 src/moving_0.cpp)
target_link_libraries(motion_planning_0 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_1 src/moving_1.cpp)
target_link_libraries(motion_planning_1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_2 src/moving_2.cpp)
target_link_libraries(motion_planning_2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_3 src/moving_3.cpp)
target_link_libraries(motion_planning_3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_4 src/moving_4.cpp)
target_link_libraries(motion_planning_4 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_5 src/moving_5.cpp)
target_link_libraries(motion_planning_5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(output_result src/output_result.cpp)
target_link_libraries(output_result ${catkin_LIBRARIES} ${Boost_LIBRARIES})
