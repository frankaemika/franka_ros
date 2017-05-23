cmake_minimum_required(VERSION 2.8.3)
project(test_franka_hw)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

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
  LIBRARIES test_franka_hw
  CATKIN_DEPENDS franka_description franka_hw controller_manager roscpp controller_interface pluginlib urdf
)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(test_franka_hw
   src/test_joint_position_limits_controller.cpp
   src/test_joint_velocity_limits_controller.cpp
   src/test_joint_torque_limits_controller.cpp)

add_dependencies(test_franka_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_franka_hw ${catkin_LIBRARIES})

add_executable(mock_franka_hw_node src/mock_franka_hw_node.cpp)
add_dependencies(mock_franka_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mock_franka_hw_node ${catkin_LIBRARIES})

## Tools
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)
include(ClangTools)
add_format_target(test_franka_hw FILES ${SOURCES} ${HEADERS})
add_tidy_target(test_franka_hw
  FILES ${SOURCES}
  DEPENDS test_franka_hw mock_franka_hw_node
)
