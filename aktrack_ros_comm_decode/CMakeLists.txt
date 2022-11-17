cmake_minimum_required(VERSION 3.5)
project(aktrack_ros_comm_decode)

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
  src/decode_node.cpp
  src/function_map_ak.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/aktrack_ros_comm_decode
  ../aktrack_ros_universal_utility/include/aktrack_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_decode_ak src/nodes/node_decode_ak.cpp)
target_link_libraries(node_decode_ak ${PROJECT_NAME})
