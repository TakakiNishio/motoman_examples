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
  tf
  geometry_msgs
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

add_executable(multiple_plans_gomi src/multiple_plans_gomi.cpp)
target_link_libraries(multiple_plans_gomi ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(multiple_plans src/multiple_plans.cpp)
target_link_libraries(multiple_plans ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(multiple_plans_and_execution src/multiple_plans_and_execution.cpp)
target_link_libraries(multiple_plans_and_execution ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(tunnel_planning_and_execution src/tunnel_planning_and_execution.cpp)
target_link_libraries(tunnel_planning_and_execution ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(go_back_to_default_pose src/go_back_to_default_pose.cpp)
target_link_libraries(go_back_to_default_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(specify_joint_angle0 src/specify_joint_angle0.cpp)
target_link_libraries(specify_joint_angle0 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(specify_joint_angle1 src/specify_joint_angle1.cpp)
target_link_libraries(specify_joint_angle1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(specify_joint_angle2 src/specify_joint_angle2.cpp)
target_link_libraries(specify_joint_angle2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(specify_joint_angle3 src/specify_joint_angle3.cpp)
target_link_libraries(specify_joint_angle3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(specify_joint_angle4 src/specify_joint_angle4.cpp)
target_link_libraries(specify_joint_angle4 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
