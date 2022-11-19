#!/bin/bash
directory="$( pwd; )";
echo [aktrack-ros INFO] Writing links to ROS workspace ...
ln -s ${directory}/aktrack_ros ${directory}/../ws_aktrack/src/aktrack_ros
echo [aktrack-ros INFO] Wrote aktrack_ros 1/6
ln -s ${directory}/aktrack_ros_comm ${directory}/../ws_aktrack/src/aktrack_ros_comm
echo [aktrack-ros INFO] Wrote aktrack_ros_comm 2/6
ln -s ${directory}/aktrack_ros_comm_decode ${directory}/../ws_aktrack/src/aktrack_ros_comm_decode
echo [aktrack-ros INFO] Wrote aktrack_ros_comm_decode 3/6
ln -s ${directory}/aktrack_ros_dispatcher ${directory}/../ws_aktrack/src/aktrack_ros_dispatcher
echo [aktrack-ros INFO] Wrote aktrack_ros_dispatcher 4/6
ln -s ${directory}/aktrack_ros_kinematics ${directory}/../ws_aktrack/src/aktrack_ros_kinematics
echo [aktrack-ros INFO] Wrote aktrack_ros_kinematics 5/6
ln -s ${directory}/aktrack_ros_universal_utility ${directory}/../ws_aktrack/src/aktrack_ros_universal_utility
echo [aktrack-ros INFO] Wrote aktrack_ros_universal_utility 6/6
echo Complete.