cmake_minimum_required(VERSION 2.8.3)

project(test_franka_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  franka_description
  franka_hw
  controller_manager
  roscpp
  controller_interface
  pluginlib
  urdf
  joint_limits_interface
  xmlrpcpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test_joint_limit_interfaces_controller
  CATKIN_DEPENDS franka_description franka_hw controller_manager roscpp controller_interface pluginlib urdf
)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(test_franka_hw src/test_joint_limit_interfaces_controller.cpp)
add_dependencies(test_franka_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_franka_hw ${catkin_LIBRARIES})

add_executable(mock_franka_hw_node src/mock_franka_hw_node.cpp)
add_dependencies(mock_franka_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mock_franka_hw_node ${catkin_LIBRARIES})
