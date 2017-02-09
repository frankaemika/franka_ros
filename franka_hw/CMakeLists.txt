cmake_minimum_required(VERSION 2.8.3)
project(franka_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
  pluginlib
  std_msgs
  sensor_msgs
  message_generation
  realtime_tools
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


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(franka_hw_node
  src/franka_hw_node.cpp
)

add_library(franka_hw
  src/franka_hw.cpp
)

add_dependencies(franka_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_hw_generate_messages_cpp
)

add_dependencies(franka_hw_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_hw_generate_messages_cpp
)

target_link_libraries(franka_hw
  ${catkin_LIBRARIES}
  Franka::franka
)

target_link_libraries(franka_hw_node
  ${catkin_LIBRARIES}
  Franka::franka
  franka_hw
)

target_include_directories(franka_hw PUBLIC
  ${catkin_INCLUDE_DIRS}
)

install(FILES franka_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

