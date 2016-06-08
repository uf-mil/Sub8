
cmake_minimum_required(VERSION 2.8.3)
project(mil_c3_trajectory_generator)

find_package(catkin REQUIRED COMPONENTS nav_msgs actionlib message_runtime message_generation tf sub8_msgs sub8_ros_tools uf_common cmake_modules)

add_service_files(
  FILES
  SetDisabled.srv
)

find_package(Eigen REQUIRED)

include_directories(
	include
	${Eigen_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

generate_messages(
    DEPENDENCIES
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS nav_msgs actionlib message_runtime message_generation tf sub8_msgs sub8_ros_tools uf_common
    INCLUDE_DIRS include
    LIBRARIES
)



add_executable(mil_c3_trajectory_generator src/C3Trajectory.cpp src/node.cpp src/AttitudeHelpers.cpp)
target_link_libraries(mil_c3_trajectory_generator ${catkin_LIBRARIES})
add_dependencies(mil_c3_trajectory_generator ${catkin_EXPORTED_TARGETS})
add_dependencies(mil_c3_trajectory_generator sub8_msgs_generate_messages_cpp)
add_dependencies(mil_c3_trajectory_generator ${PROJECT_NAME}_generate_messages_cpp)
set_target_properties(mil_c3_trajectory_generator PROPERTIES COMPILE_FLAGS -std=c++11)



