cmake_minimum_required(VERSION 2.8.3)
project(kite_tracking)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kite_tracking
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_node src/camera.cpp)
target_link_libraries(camera_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(display_node src/display.cpp)
target_link_libraries(display_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(LQR_node src/LQR.cpp)
target_link_libraries(LQR_node
  ${catkin_LIBRARIES}
)

add_executable(ptuserialcom_node src/PTUserialcom.cpp
  src/linuxser.c
  src/ptu.c
)
target_link_libraries(ptuserialcom_node
  ${catkin_LIBRARIES}
)


add_executable(singlemarkerdetection_node src/singlemarkerdetection.cpp)
target_link_libraries(singlemarkerdetection_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(arduinoserialcom_node src/arduinoserialcom.cpp)
target_link_libraries(arduinoserialcom_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)


# use c++11
add_definitions(-std=c++11)

