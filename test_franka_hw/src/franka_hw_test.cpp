#include <gtest/gtest.h>
#include <array>
#include <fstream>
#include <list>
#include <random>
#include <set>
#include <streambuf>
#include <string>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>

hardware_interface::ControllerInfo newInfo(
    std::string name,
    std::string type,
    hardware_interface::InterfaceResources resource1) {
  hardware_interface::ControllerInfo info;
  info.claimed_resources.clear();
  info.name = name;
  info.type = type;
  info.claimed_resources.push_back(resource1);
  return info;
}

hardware_interface::ControllerInfo newInfo(
    std::string name,
    std::string type,
    hardware_interface::InterfaceResources resource1,
    hardware_interface::InterfaceResources resource2) {
  hardware_interface::ControllerInfo info = newInfo(name, type, resource1);
  info.claimed_resources.push_back(resource2);
  return info;
}

TEST(FrankaHWTests, checkForConflictAndPrepareSwitchOk) {
  ros::NodeHandle nh;
  std::string arm_id("franka_emika");
  std::vector<std::string> joint_names(7);
  for (size_t i = 0; i < 7; ++i) {
    joint_names[i] = arm_id + "_joint" + std::to_string(i + 1);
  }
  franka_hw::FrankaHW robot(joint_names, nullptr, 30.0, arm_id, nh);
  std::list<hardware_interface::ControllerInfo> info_list, empty_stop_list;
  hardware_interface::InterfaceResources resource1, resource2;
  std::set<std::string> resources_set, resources_set_cart,
      resource_set_cart_other_arm;
  for (int i = 0; i < 7; ++i) {
    resources_set.insert(joint_names[i]);
  }
  resources_set_cart.insert(arm_id + "_cartesian");
  resource_set_cart_other_arm.insert("franka2_cartesian");

  // Test single claims on single arm
  // /////////////////////////////////////////////////////////////////////////////////

  // Case  no arm ID set -> should report conflict
  std::set<std::string> no_id_set;
  no_id_set.insert("joint1");
  resource1.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource1.resources = no_id_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case unknown interface -> should report conflict
  resource1.hardware_interface = "hardware_interface::UnknownDummyInterface";
  resource1.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint position -> should not report conflict
  resource1.hardware_interface = "hardware_interface::PositionJointInterface";
  resource1.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case joint velocity -> should not report conflict
  resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
  resource1.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case joint torque -> should not report conflict
  resource1.hardware_interface = "hardware_interface::EffortJointInterface";
  resource1.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case cartesian pose -> should not report conflict
  resource1.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource1.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case cartesian velocity -> should not report conflict
  resource1.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
  resource1.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Test dual claims on single arm
  // ////////////////////////////////////////////////////////////////////////////////////

  // Case joint torque and joint torque -> should report conflict
  resource1.hardware_interface = "hardware_interface::EffortJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "hardware_interface::EffortJointInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint torque and joint Position -> should not report conflict
  resource1.hardware_interface = "hardware_interface::EffortJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "hardware_interface::PositionJointInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case joint torque and joint Velocity -> should not report conflict
  resource1.hardware_interface = "hardware_interface::EffortJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "hardware_interface::VelocityJointInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case joint torque and cartesian pose -> should not report conflict
  resource1.hardware_interface = "hardware_interface::EffortJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case joint torque and cartesian velocity -> should not report conflict
  resource1.hardware_interface = "hardware_interface::EffortJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));

  // Case joint position and joint position -> should report conflict
  resource1.hardware_interface = "hardware_interface::PositionJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "hardware_interface::PositionJointInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint position and joint velocity -> should report conflict
  resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "hardware_interface::PositionJointInterface";
  resource2.resources = resources_set;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint position and cartesian pose -> should report conflict
  resource1.hardware_interface = "hardware_interface::PositionJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource2.resources = resources_set_cart;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint position and cartesian velocity -> should report conflict
  resource1.hardware_interface = "hardware_interface::PositionJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
  resource2.resources = resources_set_cart;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint velocity and cartesian velocity -> should report conflict
  resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
  resource2.resources = resources_set_cart;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint velocity and cartesian Pose -> should report conflict
  resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
  resource1.resources = resources_set;
  resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource2.resources = resources_set_cart;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case cartesian velocity and cartesian pose -> should report conflict
  resource1.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
  resource1.resources = resources_set_cart;
  resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource2.resources = resources_set_cart;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Test triple claim on single arm
  // ////////////////////////////////////////////////////////

  //  Case 3 command interfaces -> should report conflict
  resource1.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
  resource1.resources = resources_set_cart;
  hardware_interface::ControllerInfo info =
      newInfo("controller1", "dummyControllerClass", resource1, resource1);
  info.claimed_resources.push_back(resource1);
  info_list.clear();
  info_list.push_back(info);
  ASSERT_TRUE(robot.checkForConflict(info_list));

  // Test claims on two arms
  // ////////////////////////////////////////////////////////////////////
  // Case 2 arms cartesian pose -> should not report conflict
  resource1.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource1.resources = resources_set_cart;
  resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
  resource2.resources = resource_set_cart_other_arm;
  info_list.clear();
  info_list.push_back(
      newInfo("controller1", "dummyControllerClass", resource1, resource2));
  ASSERT_FALSE(robot.checkForConflict(info_list));
  ASSERT_TRUE(robot.prepareSwitch(info_list, empty_stop_list));
}

TEST(FrankaHWTests, interfacesOk) {
  ros::NodeHandle nh;
  std::string arm_id("franka_emika");
  std::vector<std::string> joint_names(7);
  for (size_t i = 0; i < 7; ++i) {
    joint_names[i] = arm_id + "_joint" + std::to_string(i + 1);
  }
  franka_hw::FrankaHW robot(joint_names, nullptr, 30.0, arm_id, nh);

  hardware_interface::JointStateInterface* js_interface =
      robot.get<hardware_interface::JointStateInterface>();
  hardware_interface::PositionJointInterface* pj_interface =
      robot.get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface* vj_interface =
      robot.get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* ej_interface =
      robot.get<hardware_interface::EffortJointInterface>();
  franka_hw::FrankaPoseCartesianInterface* fpc_interface =
      robot.get<franka_hw::FrankaPoseCartesianInterface>();
  franka_hw::FrankaVelocityCartesianInterface* fvc_interface =
      robot.get<franka_hw::FrankaVelocityCartesianInterface>();
  franka_hw::FrankaJointStateInterface* fjs_interface =
      robot.get<franka_hw::FrankaJointStateInterface>();
  franka_hw::FrankaCartesianStateInterface* fcs_interface =
      robot.get<franka_hw::FrankaCartesianStateInterface>();

  ASSERT_TRUE(js_interface != NULL);
  ASSERT_TRUE(pj_interface != NULL);
  ASSERT_TRUE(vj_interface != NULL);
  ASSERT_TRUE(ej_interface != NULL);
  ASSERT_TRUE(fpc_interface != NULL);
  ASSERT_TRUE(fvc_interface != NULL);
  ASSERT_TRUE(fjs_interface != NULL);
  ASSERT_TRUE(fcs_interface != NULL);

  // Test handle of franka_hw cartesian interfaces
  franka_hw::FrankaCartesianPoseHandle fpc_handle =
      fpc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 16> pose_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0};
  fpc_handle.setCommand(pose_command);
  ASSERT_TRUE(pose_command == fpc_handle.getCommand());

  franka_hw::FrankaCartesianVelocityHandle fvc_handle =
      fvc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 6> vel_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  fvc_handle.setCommand(vel_command);
  ASSERT_TRUE(vel_command == fvc_handle.getCommand());

  ASSERT_NO_THROW(fcs_interface->getHandle(arm_id + "_cartesian"));

  for (size_t i = 0; i < 7; ++i) {
    ASSERT_NO_THROW(fjs_interface->getHandle(joint_names[i]));
  }
}

TEST(FrankaHWTests, jointLimitInterfacesOk) {
  ros::NodeHandle nh;
  std::string arm_id("franka_emika");
  std::vector<std::string> joint_names(7);
  for (size_t i = 0; i < 7; ++i) {
    joint_names[i] = arm_id + "_joint" + std::to_string(i + 1);
  }
  franka_hw::FrankaHW robot(joint_names, nullptr, 30.0, arm_id, nh);

  hardware_interface::PositionJointInterface* pj_interface =
      robot.get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface* vj_interface =
      robot.get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* ej_interface =
      robot.get<hardware_interface::EffortJointInterface>();
  ASSERT_TRUE(pj_interface != NULL);
  ASSERT_TRUE(vj_interface != NULL);
  ASSERT_TRUE(ej_interface != NULL);

  // parse joint limits from robot description

  urdf::Model urdf_model;
  ASSERT_TRUE(urdf_model.initParamWithNodeHandle("robot_description", nh));
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

  std::uniform_real_distribution<double> uniform_distribution(0.0, 3.0);
  std::default_random_engine random_engine;

  for (size_t i = 0; i < joint_names.size(); ++i) {
    position_handles[i].setCommand(joint_limits[i].max_position +
                                   uniform_distribution(random_engine));
    velocity_handles[i].setCommand(joint_limits[i].max_velocity +
                                   uniform_distribution(random_engine));
    effort_handles[i].setCommand(joint_limits[i].max_effort +
                                 uniform_distribution(random_engine));
  }
  robot.enforceLimits(ros::Duration(0.001));
  for (size_t i = 0; i < joint_names.size(); ++i) {
    ASSERT_TRUE(
        position_handles[i].getCommand() <= joint_limits[i].max_position &&
        position_handles[i].getCommand() >= joint_limits[i].min_position);
    ASSERT_TRUE(
        velocity_handles[i].getCommand() <= joint_limits[i].max_velocity &&
        velocity_handles[i].getCommand() >= -joint_limits[i].max_velocity);
    ASSERT_TRUE(effort_handles[i].getCommand() <= joint_limits[i].max_effort &&
                effort_handles[i].getCommand() >= -joint_limits[i].max_effort);
  }

  for (size_t i = 0; i < joint_names.size(); ++i) {
    position_handles[i].setCommand(joint_limits[i].min_position -
                                   uniform_distribution(random_engine));
    velocity_handles[i].setCommand(-joint_limits[i].max_velocity -
                                   uniform_distribution(random_engine));
    effort_handles[i].setCommand(-joint_limits[i].max_effort -
                                 uniform_distribution(random_engine));
  }
  robot.enforceLimits(ros::Duration(0.001));
  for (size_t i = 0; i < joint_names.size(); ++i) {
    ASSERT_TRUE(
        position_handles[i].getCommand() <= joint_limits[i].max_position &&
        position_handles[i].getCommand() >= joint_limits[i].min_position);
    ASSERT_TRUE(
        velocity_handles[i].getCommand() <= joint_limits[i].max_velocity &&
        velocity_handles[i].getCommand() >= -joint_limits[i].max_velocity);
    ASSERT_TRUE(effort_handles[i].getCommand() <= joint_limits[i].max_effort &&
                effort_handles[i].getCommand() >= -joint_limits[i].max_effort);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "FrankaHWTestNode");
  ros::NodeHandle nh;
  std::string path = ros::package::getPath("test_franka_hw");
  std::ifstream t(path + "/urdf/franka.urdf");
  std::string urdf_string((std::istreambuf_iterator<char>(t)),
                          std::istreambuf_iterator<char>());
  nh.setParam("robot_description", urdf_string);
  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
