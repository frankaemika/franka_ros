#include <gtest/gtest.h>
#include <array>
#include <random>
#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>

extern std::string arm_id;
extern std::vector<std::string> joint_names;

namespace franka_hw {

TEST(FrankaHWTests, InterfacesWorkForReadAndCommand) {
  std::unique_ptr<franka_hw::FrankaHW> robotptr(new FrankaHW(joint_names, nullptr, nullptr, arm_id, ros::NodeHandle()));
  hardware_interface::JointStateInterface* js_interface =
      robotptr->get<hardware_interface::JointStateInterface>();
  hardware_interface::PositionJointInterface* pj_interface =
      robotptr->get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface* vj_interface =
      robotptr->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* ej_interface =
      robotptr->get<hardware_interface::EffortJointInterface>();
  franka_hw::FrankaPoseCartesianInterface* fpc_interface =
      robotptr->get<franka_hw::FrankaPoseCartesianInterface>();
  franka_hw::FrankaVelocityCartesianInterface* fvc_interface =
      robotptr->get<franka_hw::FrankaVelocityCartesianInterface>();
  franka_hw::FrankaJointStateInterface* fjs_interface =
      robotptr->get<franka_hw::FrankaJointStateInterface>();
  franka_hw::FrankaCartesianStateInterface* fcs_interface =
      robotptr->get<franka_hw::FrankaCartesianStateInterface>();

  ASSERT_NE(nullptr, js_interface);
  ASSERT_NE(nullptr, pj_interface);
  ASSERT_NE(nullptr, vj_interface);
  ASSERT_NE(nullptr, ej_interface);
  ASSERT_NE(nullptr, fpc_interface);
  ASSERT_NE(nullptr, fvc_interface);
  ASSERT_NE(nullptr, fjs_interface);
  ASSERT_NE(nullptr, fcs_interface);

  franka_hw::FrankaCartesianPoseHandle fpc_handle =
      fpc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 16> pose_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0};
  fpc_handle.setCommand(pose_command);
  EXPECT_EQ(pose_command, fpc_handle.getCommand());

  franka_hw::FrankaCartesianVelocityHandle fvc_handle =
      fvc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 6> vel_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  fvc_handle.setCommand(vel_command);
  EXPECT_EQ(vel_command, fvc_handle.getCommand());

  EXPECT_NO_THROW(fcs_interface->getHandle(arm_id + "_cartesian"));
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NO_THROW(fjs_interface->getHandle(joint_names[i]));
  }
}

TEST(FrankaHWTests, JointLimitInterfacesEnforceLimitsOnCommands) {
  std::unique_ptr<franka_hw::FrankaHW> robot_ptr(new FrankaHW(joint_names, nullptr, nullptr, arm_id, ros::NodeHandle()));

  hardware_interface::PositionJointInterface* pj_interface =
      robot_ptr->get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface* vj_interface =
      robot_ptr->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* ej_interface =
      robot_ptr->get<hardware_interface::EffortJointInterface>();
  ASSERT_NE(nullptr, pj_interface);
  ASSERT_NE(nullptr, vj_interface);
  ASSERT_NE(nullptr, ej_interface);

  urdf::Model urdf_model;
  ASSERT_TRUE(urdf_model.initParamWithNodeHandle("robot_description", ros::NodeHandle()));
  std::vector<joint_limits_interface::JointLimits> joint_limits(7);
  std::vector<hardware_interface::JointHandle> position_handles(7);
  std::vector<hardware_interface::JointHandle> velocity_handles(7);
  std::vector<hardware_interface::JointHandle> effort_handles(7);

  for (size_t i = 0; i < joint_names.size(); ++i) {
    boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_names[i]);
    ASSERT_TRUE(
        joint_limits_interface::getJointLimits(urdf_joint, joint_limits[i]));
    ASSERT_NO_THROW(position_handles[i] =
                        pj_interface->getHandle(joint_names[i]));
    ASSERT_NO_THROW(velocity_handles[i] =
                        vj_interface->getHandle(joint_names[i]));
    ASSERT_NO_THROW(effort_handles[i] =
                        ej_interface->getHandle(joint_names[i]));
  }

  std::uniform_real_distribution<double> uniform_distribution(0.1, 3.0);
  std::default_random_engine random_engine;

  for (size_t i = 0; i < joint_names.size(); ++i) {
    position_handles[i].setCommand(joint_limits[i].max_position +
                                   uniform_distribution(random_engine));
    velocity_handles[i].setCommand(joint_limits[i].max_velocity +
                                   uniform_distribution(random_engine));
    effort_handles[i].setCommand(joint_limits[i].max_effort +
                                 uniform_distribution(random_engine));
  }
  robot_ptr->enforceLimits(ros::Duration(0.001));
  for (size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_LE(position_handles[i].getCommand(), joint_limits[i].max_position);
    EXPECT_GE(position_handles[i].getCommand(),joint_limits[i].min_position);
    EXPECT_LE(velocity_handles[i].getCommand(), joint_limits[i].max_velocity);
    EXPECT_GE(velocity_handles[i].getCommand(), -joint_limits[i].max_velocity);
    EXPECT_LE(effort_handles[i].getCommand(), joint_limits[i].max_effort);
    EXPECT_GE(effort_handles[i].getCommand(), -joint_limits[i].max_effort);
  }

  for (size_t i = 0; i < joint_names.size(); ++i) {
    position_handles[i].setCommand(joint_limits[i].min_position -
                                   uniform_distribution(random_engine));
    velocity_handles[i].setCommand(-joint_limits[i].max_velocity -
                                   uniform_distribution(random_engine));
    effort_handles[i].setCommand(-joint_limits[i].max_effort -
                                 uniform_distribution(random_engine));
  }
  robot_ptr->enforceLimits(ros::Duration(0.001));
  for (size_t i = 0; i < joint_names.size(); ++i) {
      EXPECT_LE(position_handles[i].getCommand(), joint_limits[i].max_position);
      EXPECT_GE(position_handles[i].getCommand(),joint_limits[i].min_position);
      EXPECT_LE(velocity_handles[i].getCommand(), joint_limits[i].max_velocity);
      EXPECT_GE(velocity_handles[i].getCommand(), -joint_limits[i].max_velocity);
      EXPECT_LE(effort_handles[i].getCommand(), joint_limits[i].max_effort);
      EXPECT_GE(effort_handles[i].getCommand(), -joint_limits[i].max_effort); }
}

}  // namespace franka_hw

