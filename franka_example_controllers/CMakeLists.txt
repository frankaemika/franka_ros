cmake_minimum_required(VERSION 2.8.3)
project(franka_example_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  roscpp
  sensor_msgs
  geometry_msgs
  realtime_tools
  tf2_msgs
  tf
  franka_description
  franka_hw
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_example_controllers
  CATKIN_DEPENDS controller_interface franka_description franka_hw pluginlib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(franka_example_controllers 
  src/joint_velocity_example_controller.cpp
  src/joint_position_example_controller.cpp
  src/cartesian_pose_example_controller.cpp
  src/cartesian_velocity_example_controller.cpp
  src/franka_state_controller.cpp)

add_dependencies(franka_example_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(franka_example_controllers ${catkin_LIBRARIES})

## Tools
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(ClangTools)
add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
add_tidy_target(franka_example_controllers
  FILES ${SOURCES}
  DEPENDS franka_example_controllers
)
