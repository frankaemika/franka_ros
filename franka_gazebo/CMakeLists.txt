cmake_minimum_required(VERSION 2.8.3)
project(franka_gazebo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  gazebo_ros_control
  kdl_parser
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  franka_hw
)

find_package(Franka 0.7.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    gazebo_ros_control
    kdl_parser
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
    franka_hw
  INCLUDE_DIRS include
  LIBRARIES franka_hw_sim
  DEPENDS Franka orocos_kdl
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(franka_hw_sim
  src/franka_hw_sim.cpp
  src/joint.cpp
  src/model_kdl.cpp
  )
target_link_libraries(franka_hw_sim
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  )
target_include_directories(franka_hw_sim SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
add_dependencies(franka_hw_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

## Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES franka_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
