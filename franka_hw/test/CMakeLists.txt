find_package(rostest REQUIRED)

add_rostest_gtest(franka_hw_controller_switching_test
  launch/franka_hw_controller_switching_test.test
  franka_hw_controller_switching_test.cpp
)

add_dependencies(franka_hw_controller_switching_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_controller_switching_test
  ${catkin_LIBRARIES}
  Franka::franka
  franka_hw
)

target_include_directories(franka_hw_controller_switching_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(franka_hw_interfaces_test
  launch/franka_hw_interfaces_test.test
  franka_hw_interfaces_test.cpp
)

add_dependencies(franka_hw_interfaces_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_interfaces_test
  ${catkin_LIBRARIES}
  Franka::franka
  franka_hw
)

target_include_directories(franka_hw_interfaces_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)
