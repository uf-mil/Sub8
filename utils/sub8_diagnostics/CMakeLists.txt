cmake_minimum_required(VERSION 2.8.3)
project(sub8_diagnostics)

find_package(catkin
 REQUIRED COMPONENTS
  roscpp
  rospy
  mil_tools
  message_runtime
  sub8_msgs
  mil_msgs
  actionlib
  message_generation
  geometry_msgs
  sensor_msgs
  interactive_markers
  std_msgs
  actionlib_msgs
  mil_tools
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs geometry_msgs sensor_msgs
)


# Any tools used to diagnose the core functionality of the sub should be put here
catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS
  actionlib
  message_runtime
  mil_msgs
  mil_tools
  roscpp
  sub8_msgs
  message_generation
  message_runtime
  geometry_msgs
  std_msgs
  actionlib_msgs
 DEPENDS
  mil_msgs
)







include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(move_test src/move_test.cpp)

target_link_libraries(
  move_test
  ${catkin_LIBRARIES}
)

add_dependencies(
  move_test
  ${mil_msgs_EXPORTED_TARGETS}
)
