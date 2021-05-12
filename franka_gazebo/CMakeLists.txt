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
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

find_package(Franka 0.7.0 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    gazebo_ros_control
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
  INCLUDE_DIRS include
  LIBRARIES franka_hw_sim
  DEPENDS Franka
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(franka_hw_sim src/franka_hw_sim.cpp)
target_link_libraries(franka_hw_sim
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  )
target_include_directories(franka_hw_sim SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
add_dependencies(franka_hw_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES franka_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
