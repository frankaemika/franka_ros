cmake_minimum_required(VERSION 2.8.3)
project(franka_example_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_description
  franka_hw
  pluginlib
  realtime_tools
  roscpp
  message_generation
  dynamic_reconfigure
)

add_message_files(FILES
  JointTorqueComparison.msg
)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_example_controllers
  CATKIN_DEPENDS controller_interface franka_description franka_hw pluginlib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(franka_example_controllers
  src/cartesian_pose_example_controller.cpp
  src/cartesian_velocity_example_controller.cpp
  src/joint_position_example_controller.cpp
  src/joint_velocity_example_controller.cpp
  src/model_example_controller.cpp
  src/joint_impedance_example_controller.cpp
  src/cartesian_impedance_example_controller.cpp
  src/force_example_controller.cpp
)

add_dependencies(franka_example_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_example_controllers ${catkin_LIBRARIES})

## Installation
install(TARGETS franka_example_controllers
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
install(FILES franka_example_controllers_plugin.xml
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
  add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_example_controllers
    FILES ${SOURCES}
    DEPENDS franka_example_controllers
  )
endif()
