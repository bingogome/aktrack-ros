cmake_minimum_required(VERSION 3.5)
project(aktrack_ros_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
  tf2
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
  src/transform_conversions.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC 
  include/aktrack_ros_kinematics
  ../aktrack_ros_universal_utility/include/aktrack_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_calibration_data src/nodes/node_calibration_data.cpp)
target_link_libraries(node_calibration_data ${PROJECT_NAME})

add_executable(node_dataac_t_sticker_trackercent src/nodes/node_dataac_t_sticker_trackercent.cpp)
target_link_libraries(node_dataac_t_sticker_trackercent ${PROJECT_NAME})

add_executable(node_t_sticker_trackercent src/nodes/node_t_sticker_trackercent.cpp)
target_link_libraries(node_t_sticker_trackercent ${PROJECT_NAME})

add_executable(node_viz_t_sticker_trackercent src/nodes/node_viz_t_sticker_trackercent.cpp)
target_link_libraries(node_viz_t_sticker_trackercent ${PROJECT_NAME})