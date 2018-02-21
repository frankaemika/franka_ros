cmake_minimum_required(VERSION 2.8.3)
project(franka_visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
)

find_package(Franka 0.3.0 REQUIRED)

catkin_package(CATKIN_DEPENDS sensor_msgs roscpp)

set(EXECUTABLES robot_joint_state_publisher gripper_joint_state_publisher)

foreach(executable ${EXECUTABLES})
  add_executable(${executable}
    src/${executable}.cpp
  )
  add_dependencies(${executable}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(${executable} SYSTEM PUBLIC
    ${Franka_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${executable} PUBLIC
    ${Franka_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endforeach()

## Installation
install(TARGETS ${EXECUTABLES}
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
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_visualization FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_visualization
    FILES ${SOURCES}
    DEPENDS ${EXECUTABLES}
  )
endif()
