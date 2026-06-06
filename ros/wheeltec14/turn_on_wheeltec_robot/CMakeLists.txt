cmake_minimum_required(VERSION 2.8.3)
project(turn_on_wheeltec_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

add_compile_options(-g -std=c++11)

add_message_files(
  FILES
  AkmState.msg
  ControlDebug.msg
  ChassisDiagnostics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})

add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
add_dependencies(wheeltec_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})

add_executable(twist_transform src/twist_transform.cpp)
target_link_libraries(twist_transform ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/launch_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
