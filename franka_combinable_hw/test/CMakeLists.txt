find_package(rostest REQUIRED)

add_rostest_gtest(franka_combinable_hw_test
  launch/franka_combinable_hw_test.test
  main.cpp
  franka_combinable_hw_controller_switching_test.cpp
)

add_dependencies(franka_combinable_hw_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_combinable_hw_test
  ${catkin_LIBRARIES}
  Franka::Franka
  franka_combinable_hw
)

target_include_directories(franka_combinable_hw_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)

