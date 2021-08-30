cmake_minimum_required(VERSION 3.4)
project(franka_vacuum_gripper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  control_msgs
  actionlib
  sensor_msgs
  xmlrpcpp
  actionlib_msgs
)

find_package(Franka 0.7.0 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES Vacuum.action
        Stop.action
	DropOff.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_vacuum_gripper
  CATKIN_DEPENDS roscpp
                 message_runtime
                 control_msgs
                 actionlib
                 sensor_msgs
                 xmlrpcpp
                 actionlib_msgs
  DEPENDS Franka
)

add_library(franka_vacuum_gripper
  src/franka_vacuum_gripper.cpp
)

add_dependencies(franka_vacuum_gripper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(franka_vacuum_gripper
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_vacuum_gripper SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_vacuum_gripper PUBLIC
  include
)

add_executable(franka_vacuum_gripper_node
  src/franka_vacuum_gripper_node.cpp
)

add_dependencies(franka_vacuum_gripper_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_vacuum_gripper
)

target_link_libraries(franka_vacuum_gripper_node
   ${catkin_LIBRARIES}
   franka_vacuum_gripper
)

target_include_directories(franka_vacuum_gripper_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

## Installation
install(TARGETS franka_vacuum_gripper
                franka_vacuum_gripper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_vacuum_gripper FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_vacuum_gripper
    FILES ${SOURCES}
    DEPENDS franka_vacuum_gripper franka_vacuum_gripper_node
  )
endif()
