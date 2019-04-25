cmake_minimum_required(VERSION 3.4)
project(franka_combinable_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  combined_robot_hw
  controller_interface
  franka_hw
  franka_control
  hardware_interface
  joint_limits_interface
  pluginlib
  roscpp
  std_msgs
)

find_package(Franka 0.5.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_combinable_hw combined_robot_hw
  CATKIN_DEPENDS
    actionlib
    combined_robot_hw
    controller_interface
    franka_hw
    franka_control
    hardware_interface
    joint_limits_interface
    pluginlib
    roscpp
    std_msgs
  DEPENDS Franka
)

add_library(franka_combinable_hw
  src/franka_combinable_hw.cpp
  src/franka_combined_hw.cpp
)

add_dependencies(franka_combinable_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_combinable_hw
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_combinable_hw SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_combinable_hw PUBLIC
  include
)

## Installation
install(TARGETS franka_combinable_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES franka_combinable_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
  add_format_target(franka_combinable_hw FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_combinable_hw
    FILES ${SOURCES}
    DEPENDS franka_combinable_hw
  )
endif()
