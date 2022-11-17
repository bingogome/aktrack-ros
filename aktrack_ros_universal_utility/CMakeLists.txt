cmake_minimum_required(VERSION 3.5)
project(aktrack_ros_universal_utility)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  )

catkin_package(
  CATKIN_DEPENDS roscpp
)

add_library(${PROJECT_NAME} 
  src/dummy.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/aktrack_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})