find_package(rostest REQUIRED)

add_rostest_gtest(franka_hw_sim_test
  launch/franka_hw_sim.test
  main.cpp
  jacobian_tests.cpp
  fk_tests.cpp
  gravity_tests.cpp
)

add_dependencies(franka_hw_sim_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_sim_test
  ${catkin_LIBRARIES}
  franka_hw_sim
)

target_include_directories(franka_hw_sim_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(franka_hw_sim_gazebo_test
  launch/franka_hw_sim_gazebo.test
  franka_hw_sim_gazebo_test.cpp
)

add_dependencies(franka_hw_sim_gazebo_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_sim_gazebo_test
  ${CATKIN_LIBRARIES}
  franka_hw_sim
)

target_include_directories(franka_hw_sim_gazebo_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(franka_gripper_sim_test launch/franka_gripper_sim.test
  franka_gripper_sim_test.cpp
  gripper_sim_test_setup.cpp
  gripper_sim_test_setup.h
)

add_dependencies(franka_gripper_sim_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_gripper_sim_test
  ${CATKIN_LIBRARIES}
  franka_gripper_sim
)

target_include_directories(franka_gripper_sim_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(franka_gripper_sim_test_with_object
  launch/franka_gripper_sim_with_object.test
  franka_gripper_sim_test_with_object.cpp
  gripper_sim_test_setup.cpp
  gripper_sim_test_setup.h
)

add_dependencies(franka_gripper_sim_test_with_object
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_gripper_sim_test_with_object
  ${CATKIN_LIBRARIES}
  franka_gripper_sim
)

target_include_directories(franka_gripper_sim_test_with_object PUBLIC
  ${catkin_INCLUDE_DIRS}
)

catkin_add_gtest(franka_hw_sim_controller_verifier_test
  main.cpp
  controller_verifier_test.cpp
)

add_dependencies(franka_hw_sim_controller_verifier_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_sim_controller_verifier_test
  ${catkin_LIBRARIES}
  franka_hw_sim
)

target_include_directories(franka_hw_sim_controller_verifier_test PUBLIC
  ${catkin_INCLUDE_DIRS}
)