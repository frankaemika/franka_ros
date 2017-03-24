cmake_minimum_required(VERSION 2.8.3)
project(franka_visualization)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
)

find_package(Franka REQUIRED)

catkin_package(CATKIN_DEPENDS sensor_msgs roscpp)

add_library(forward_kinematics STATIC IMPORTED)
set_property(TARGET forward_kinematics PROPERTY IMPORTED_LOCATION
      /home/jaeh_ch/Code/franka_dynamics_kinematics/fci_libs_64/O_T_J9_64.a)


add_executable(franka_joint_state_publisher
  src/franka_joint_state_publisher.cpp
)

add_dependencies(franka_joint_state_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_joint_state_publisher
  ${catkin_LIBRARIES}
  Franka::franka
)
target_include_directories(franka_joint_state_publisher PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_executable(franka_cartesian_motion_example
  src/franka_cartesian_motion_example.cpp
)

add_dependencies(franka_cartesian_motion_example
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  forward_kinematics
)

target_link_libraries(franka_cartesian_motion_example
  ${catkin_LIBRARIES}
  Franka::franka
  forward_kinematics
)

target_include_directories(franka_cartesian_motion_example PUBLIC
  ${catkin_INCLUDE_DIRS}
  /home/jaeh_ch/Code/franka_dynamics_kinematics/fci_libs_64
)

add_executable(franka_joint_motion_example
  src/franka_joint_motion_example.cpp
)

add_dependencies(franka_joint_motion_example
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_joint_motion_example
  ${catkin_LIBRARIES}
  Franka::franka
)

target_include_directories(franka_joint_motion_example PUBLIC
  ${catkin_INCLUDE_DIRS}
)

## Tools
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)
include(ClangTools)
add_format_target(franka_visualization FILES ${SOURCES} ${HEADERS})
add_tidy_target(franka_visualization
  FILES ${SOURCES}
  DEPENDS franka_joint_state_publisher franka_motion_generator_example
)
