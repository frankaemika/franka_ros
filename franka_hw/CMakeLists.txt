cmake_minimum_required(VERSION 2.8.3)
project(franka_hw)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  realtime_tools
  tf2_msgs
  tf
  urdf
  joint_limits_interface
  xmlrpcpp
)

find_package(Franka REQUIRED)

add_message_files(
  FILES
  FrankaState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_hw
  CATKIN_DEPENDS hardware_interface roscpp message_runtime realtime_tools
  DEPENDS Franka
)

add_library(franka_hw
  src/franka_hw.cpp
  src/trigger_rate.cpp
)

add_dependencies(franka_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_hw_generate_messages_cpp
)

target_link_libraries(franka_hw
  ${catkin_LIBRARIES}
  Franka::franka
)

target_include_directories(franka_hw PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(franka_hw_node
  src/franka_hw_node.cpp
)

add_dependencies(franka_hw_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_hw_generate_messages_cpp
)

target_link_libraries(franka_hw_node
  ${catkin_LIBRARIES}
  franka_hw
)

target_include_directories(franka_hw_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)

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
