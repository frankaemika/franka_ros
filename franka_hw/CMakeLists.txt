cmake_minimum_required(VERSION 2.8.3)
project(franka_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
)

find_package(Franka REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_hw
  CATKIN_DEPENDS hardware_interface roscpp
  DEPENDS Franka
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(franka_hw
   src/franka_hw.cpp
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
)

install(FILES franka_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

