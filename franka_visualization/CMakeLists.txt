cmake_minimum_required(VERSION 2.8.3)
project(franka_visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
)

#HACK
#list(INSERT CMAKE_MODULE_PATH 0 /home/jaeh_ch/Code/libfranka/cmake)
#find_package(Poco REQUIRED COMPONENTS Net Foundation)
#set(Franka_DIR "/home/jaeh_ch/Code/libfranka/build/")
find_package(Franka REQUIRED)

catkin_package(CATKIN_DEPENDS sensor_msgs roscpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
#  /home/jaeh_ch/Code/libfranka/include
)

add_executable(simple_viz_node
  src/simple_viz_node.cpp
)

add_dependencies(simple_viz_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(simple_viz_node
   ${catkin_LIBRARIES}
   Franka::franka
)
target_include_directories(simple_viz_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)
