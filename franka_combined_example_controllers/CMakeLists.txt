cmake_minimum_required(VERSION 3.4)
project(franka_combined_example_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_combinable_hw
  franka_example_controllers
  franka_hw
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  pluginlib
  realtime_tools
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/dual_arm_compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_combined_example_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_combinable_hw
    franka_example_controllers
    franka_hw
    geometry_msgs
    tf
    tf_conversions  hardware_interface
    pluginlib
    roscpp
    realtime_tools
  DEPENDS Franka
)

add_library(franka_combined_example_controllers
   src/dual_arm_cartesian_impedance_example_controller.cpp
)

target_include_directories(franka_combined_example_controllers PUBLIC
  include
)

target_include_directories(franka_combined_example_controllers SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_dependencies(franka_combined_example_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_combined_example_controllers PUBLIC
   ${catkin_LIBRARIES}
   ${Franka_LIBRARIES}
)

## Installation
install(TARGETS franka_combined_example_controllers
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
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_combined_example_controllers_plugin.xml
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
  add_format_target(franka_combined_example_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_combined_example_controllers
    FILES ${SOURCES}
    DEPENDS franka_combined_example_controllers
  )
endif()
