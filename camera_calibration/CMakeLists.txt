cmake_minimum_required(VERSION 3.18)
project("camera_calibration")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs roslib)

find_package(OpenCV 4.0 REQUIRED)
find_package(Boost 1.65 COMPONENTS filesystem) 


# add_subdirectory(scripts)

set(include_headers include/)
include_directories(  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${include_headers})


add_subdirectory(src)


set_property(GLOBAL PROPERTY USE_FOLDERS ON)