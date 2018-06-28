cmake_minimum_required(VERSION 2.8.3)
project(franka_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  joint_limits_interface
  roscpp
)

find_package(Franka 0.4.0 REQUIRED)

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
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_hw SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_hw PUBLIC
  include
)

## Installation
install(TARGETS franka_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_hw FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_hw
    FILES ${SOURCES}
    DEPENDS franka_hw
  )
endif()
