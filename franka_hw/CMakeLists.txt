cmake_minimum_required(VERSION 2.8.3)
project(franka_hw)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_description
  hardware_interface
  joint_limits_interface
  roscpp
)

find_package(Franka REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_hw
  CATKIN_DEPENDS controller_interface hardware_interface roscpp
  DEPENDS Franka
)

add_library(franka_hw
  src/control_mode.cpp
  src/franka_hw.cpp
  src/resource_helpers.cpp
  src/trigger_rate.cpp
)

add_dependencies(franka_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw
  ${catkin_LIBRARIES}
  Franka::Franka
)

target_include_directories(franka_hw PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
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
  DEPENDS franka_hw
)
