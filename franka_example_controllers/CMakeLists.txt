cmake_minimum_required(VERSION 2.8.3)
project(franka_example_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
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

add_library(franka_example_controllers src/joint_velocity_controller_example.cpp)

add_dependencies(franka_example_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(franka_example_controllers ${catkin_LIBRARIES})
