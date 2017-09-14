cmake_minimum_required(VERSION 2.8.3)
project(franka_control)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_interface
  controller_manager
  franka_description
  franka_hw
  franka_msgs
  geometry_msgs
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf
  tf2_msgs
)

find_package(Franka REQUIRED)

add_service_files(FILES
  SetCartesianImpedance.srv
  SetEEFrame.srv
  SetForceTorqueCollisionBehavior.srv
  SetFullCollisionBehavior.srv
  SetJointImpedance.srv
  SetKFrame.srv
  SetLoad.srv
)

add_action_files(FILES
  ErrorRecovery.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_state_controller franka_control_services
  CATKIN_DEPENDS
    actionlib
    controller_interface
    franka_hw
    franka_msgs
    geometry_msgs
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    tf2_msgs
  DEPENDS Franka
)

## franka_state_controller
add_library(franka_state_controller
  src/franka_state_controller.cpp
)

add_dependencies(franka_state_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_state_controller
  ${catkin_LIBRARIES}
  Franka::Franka
)

target_include_directories(franka_state_controller PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

## franka_control_services
add_library(franka_control_services
  src/services.cpp
)

add_dependencies(franka_control_services
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_control_services
  ${catkin_LIBRARIES}
  Franka::Franka
)

target_include_directories(franka_control_services PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

## franka_control_node
add_executable(franka_control_node
  src/franka_control_node.cpp
)

add_dependencies(franka_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_control_node
  ${catkin_LIBRARIES}
  franka_control_services
  Franka::Franka
)

target_include_directories(franka_control_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)

## Tools
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)
include(ClangTools)
add_format_target(franka_control FILES ${SOURCES} ${HEADERS})
add_tidy_target(franka_control
  FILES ${SOURCES}
  DEPENDS franka_control_node franka_state_controller
)
