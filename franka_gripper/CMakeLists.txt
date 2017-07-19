cmake_minimum_required(VERSION 2.8.3)
project(franka_gripper)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  control_msgs
  actionlib
)

find_package(Franka REQUIRED)

add_service_files(FILES
   Stop.srv
   Move.srv
   Grasp.srv
   Homing.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_gripper
  CATKIN_DEPENDS roscpp message_runtime control_msgs actionlib
  DEPENDS Franka
)

add_library(franka_gripper
src/franka_gripper.cpp
)

add_dependencies(franka_gripper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(franka_gripper
   ${catkin_LIBRARIES}
   Franka::franka
)

target_include_directories(franka_gripper PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(franka_gripper_node
  src/franka_gripper_node.cpp
)

add_dependencies(franka_gripper_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  franka_gripper
)

target_link_libraries(franka_gripper_node
   ${catkin_LIBRARIES}
   franka_gripper
)

target_include_directories(franka_gripper_node PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

## Tools
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)
include(ClangTools)
add_format_target(franka_gripper FILES ${SOURCES} ${HEADERS})
add_tidy_target(franka_gripper
  FILES ${SOURCES}
  DEPENDS franka_gripper franka_gripper_node
)
