cmake_minimum_required(VERSION 2.8.3)
project(franka_hw)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  controller_interface
  controller_manager
  franka_description
  geometry_msgs
  hardware_interface
  joint_limits_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_msgs
  urdf
  xmlrpcpp
)

find_package(Franka REQUIRED)

add_message_files(FILES
  Errors.msg
  FrankaState.msg
)

add_service_files(FILES
  SetCartesianImpedance.srv
  SetEEFrame.srv
  SetForceTorqueCollisionBehavior.srv
  SetFullCollisionBehavior.srv
  SetJointImpedance.srv
  SetKFrame.srv
  SetLoad.srv
  SetTimeScalingFactor.srv
)

add_action_files(FILES
  ErrorRecovery.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_hw franka_state_controller
  CATKIN_DEPENDS actionlib_msgs controller_interface pluginlib
                 hardware_interface message_runtime realtime_tools roscpp
  DEPENDS Franka
)

add_library(franka_hw_services src/services.cpp)

add_dependencies(franka_hw_services
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(franka_hw_services
   ${catkin_LIBRARIES}
   Franka::franka
)

target_include_directories(franka_hw_services PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(franka_hw
  src/franka_hw.cpp
  src/franka_controller_switching_types.cpp
  src/franka_hw_helper_functions.cpp
  src/trigger_rate.cpp
)

add_dependencies(franka_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw
  ${catkin_LIBRARIES}
  Franka::franka
)

target_include_directories(franka_hw PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(franka_state_controller
  src/franka_state_controller.cpp
)

add_dependencies(franka_state_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(franka_state_controller
  ${catkin_LIBRARIES}
  Franka::franka
)

target_include_directories(franka_state_controller PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(franka_hw_node
  src/franka_hw_node.cpp
)

add_dependencies(franka_hw_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_hw_services
)

target_link_libraries(franka_hw_node
  ${catkin_LIBRARIES}
  franka_hw
  franka_hw_services
)

target_include_directories(franka_hw_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

## Tools
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)
include(ClangTools)
add_format_target(franka_hw FILES ${SOURCES} ${HEADERS})
add_tidy_target(franka_hw
  FILES ${SOURCES}
  DEPENDS franka_hw franka_hw_node
)
