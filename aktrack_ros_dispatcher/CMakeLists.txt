cmake_minimum_required(VERSION 3.5)
project(aktrack_ros_dispatcher)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  aktrack_ros_universal_utility
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp aktrack_ros_universal_utility
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/aktrack_dispatcher.cpp
  src/dispatcher_utility.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/aktrack_ros_dispatcher
  ../aktrack_ros_universal_utility/include/aktrack_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_dispatcher src/nodes/node_dispatcher.cpp)
target_link_libraries(node_dispatcher ${PROJECT_NAME})
