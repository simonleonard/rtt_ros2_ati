cmake_minimum_required(VERSION 2.8.3)
project(ati_rtt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_conversions
  kdl_parser
  roscpp
  rtt_ros
  rtt_rosdeployment
  sensor_msgs
  tf
  )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( orocos_kdl REQUIRED )
find_package( COMEDI REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${COMEDI_INCLUDE_DIR}
  include )

set( SRC_FILES_ATI
  src/dom.c
  src/ftconfig.c
  src/ftrt.c
  src/expatls.c
  src/node.c
  src/stack.c
  src/xmlparse.c
  src/xmlrole.c
  src/xmltok.c )
add_library( ati ${SRC_FILES_ATI} )

orocos_component( ati_rtt src/ati_rtt.cpp )
target_link_libraries( ati_rtt ${catkin_LIBRAIRIES} ${orocos_kdl_LIBRARIES} ${COMEDI_LIBRARY} ati )

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS
  ${PROJECT_NAME}
  )

#add_dependencies( example_node ${PROJECT_NAME}_gencfg)
#orocos_generate_package()
