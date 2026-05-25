cmake_minimum_required(VERSION 3.0.2)
project(task_manager_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(task_manager_node src/task_manager.cpp)
target_link_libraries(task_manager_node ${catkin_LIBRARIES})
