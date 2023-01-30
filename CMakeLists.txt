cmake_minimum_required(VERSION 3.0.2)
project(object_analysis)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        Original_msgs
        cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_analysis
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

add_executable(Object_detection_node
        src/Object_detection.cpp
        include/Object_detection.h
        )

target_link_libraries(Object_detection_node
        ${catkin_LIBRARIES}
        )

include_directories(
# include
        ${catkin_INCLUDE_DIRS}
)
