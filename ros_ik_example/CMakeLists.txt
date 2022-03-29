cmake_minimum_required(VERSION 3.0.2)
project(ros_ik_example)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             message_generation
             roscpp
             rospy
             sensor_msgs
             std_msgs
             tf2)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${eigen_INCLUDE_DIRS})
