cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_ati)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(ament_cmake REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_node REQUIRED)
find_package(rtt_ros2_geometry_msgs REQUIRED)
find_package(rtt_ros2_sensor_msgs REQUIRED)
find_package( COMEDI REQUIRED )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


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

add_library( ati SHARED ${SRC_FILES_ATI} )

ament_export_targets(ati HAS_LIBRARY_TARGET)
#ament_export_dependencies(

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ati
  EXPORT ati
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

orocos_component(rtt_ros2_ati src/rtt_ros2_ati.cpp)
target_link_libraries(rtt_ros2_ati
  ${COMEDI_LIBRARY}
  rtt_ros2::rtt_ros2
  rtt_ros2_node::rtt_ros2_node
  ati  
  )

orocos_generate_package()
