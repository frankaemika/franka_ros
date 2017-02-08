cmake_minimum_required(VERSION 2.8.3)
project(franka_visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
)

find_package(Franka REQUIRED)

catkin_package(CATKIN_DEPENDS sensor_msgs roscpp)

add_executable(franka_joint_state_publisher
  src/franka_joint_state_publisher.cpp
)

add_dependencies(franka_joint_state_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_joint_state_publisher
  ${catkin_LIBRARIES}
  Franka::franka
)
target_include_directories(franka_joint_state_publisher PUBLIC
  ${catkin_INCLUDE_DIRS}
)

## Tools
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ClangTools.cmake)
