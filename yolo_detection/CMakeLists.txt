cmake_minimum_required(VERSION 3.0.2)
project(yolo_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  rostime
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Object.msg
)

add_service_files(
  FILES
  Task.srv
  Target.srv
  GetPoint.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolo_detection
  CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy rostime std_msgs message_runtime
 # DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/yolo_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/yolo_detection_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


catkin_install_python(PROGRAMS
  scripts/object_service.py
  scripts/point_cloud_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#catkin_install_python(PROGRAMS  
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

catkin_install_python(PROGRAMS
  scripts/target_service_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
