cmake_minimum_required(VERSION 3.4)
project(franka_combined_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  franka_combinable_hw
  roscpp
)

find_package(Franka 0.5.0 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    controller_manager
    franka_combinable_hw
    roscpp
)

add_executable(franka_combined_control_node
    src/franka_combined_control_node.cpp
)

add_dependencies(franka_combined_control_node
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_combined_control_node
  ${catkin_LIBRARIES}
)

target_include_directories(franka_combined_control_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

## Installation
install(TARGETS franka_combined_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/franka_combined_control_node.cpp)
  add_format_target(franka_combined_control FILES ${SOURCES})
  add_tidy_target(franka_combined_control
    FILES ${SOURCES}
    DEPENDS franka_combined_control_node
  )
endif()
