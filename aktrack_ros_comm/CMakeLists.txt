cmake_minimum_required(VERSION 3.5)
project(aktrack_ros_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
  aktrack_ros_universal_utility
  )

find_package(Boost REQUIRED COMPONENTS system)

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
  src/ros_side_in.cpp
  src/ros_side_out.cpp
  src/ros_side_in_node.cpp
  src/ros_side_out_node.cpp
  src/ros_side_hif_out_node.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC 
  include/aktrack_ros_comm
  ../aktrack_ros_universal_utility/include/aktrack_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_ak_comm_in src/nodes/node_ak_comm_in.cpp)
target_link_libraries(node_ak_comm_in ${PROJECT_NAME})

add_executable(node_ak_comm_out src/nodes/node_ak_comm_out.cpp)
target_link_libraries(node_ak_comm_out ${PROJECT_NAME})

add_executable(node_comm_end src/nodes/node_comm_end.cpp)
target_link_libraries(node_comm_end ${PROJECT_NAME})

add_executable(node_ak_comm_hif_out src/nodes/node_ak_comm_hif_out.cpp)
target_link_libraries(node_ak_comm_hif_out ${PROJECT_NAME})