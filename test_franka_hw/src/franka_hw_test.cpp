#include <list>
#include <set>
#include <string>
#include <array>
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>


hardware_interface::ControllerInfo newInfo(std::string name, std::string type, hardware_interface::InterfaceResources resource1) {
  hardware_interface::ControllerInfo info;
  info.claimed_resources.clear();
  info.name = name;
  info.type = type;
  info.claimed_resources.push_back(resource1);
  return info;
}

hardware_interface::ControllerInfo newInfo(std::string name, std::string type, hardware_interface::InterfaceResources resource1, hardware_interface::InterfaceResources resource2) {
  hardware_interface::ControllerInfo info = newInfo(name, type, resource1);
  info.claimed_resources.push_back(resource2);
  return info;
}


TEST(FrankaHWTests, checkForConflictOk)
{
  ros::NodeHandle nh;
  std::string arm_id("franka");
  std::vector<std::string> joint_names(7);
  for (size_t i = 0; i < 7; ++i) {
    joint_names[i] = arm_id + "_joint" + std::to_string(i);
  }
  franka_hw::FrankaHW robot(joint_names, nullptr, 30.0, arm_id, nh);

  std::list<hardware_interface::ControllerInfo> info_list;

  hardware_interface::InterfaceResources resource1, resource2;
  std::set<std::string> resources_set, resources_set_cart, resource_set_cart_other_arm;
  for (int i=0; i<7; ++i) {
     resources_set.insert(joint_names[i]);
  }
  resources_set_cart.insert("franka_cartesian");
  resource_set_cart_other_arm.insert("franka2_cartesian");

// Test single claims on single arm /////////////////////////////////////////////////////////////////////////////////

  // Case  no arm ID set -> should report conflict
    std::set<std::string> no_id_set;
    no_id_set.insert("joint1");
    resource1.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource1.resources = no_id_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case unknown interface -> should report conflict
    resource1.hardware_interface = "hardware_interface::UnknownDummyInterface";
    resource1.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_TRUE(robot.checkForConflict(info_list));

  // Case joint position -> should not report conflict
    resource1.hardware_interface = "hardware_interface::PositionJointInterface";
    resource1.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_FALSE(robot.checkForConflict(info_list));

  // Case joint velocity -> should not report conflict
    resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
    resource1.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_FALSE(robot.checkForConflict(info_list));

  // Case joint torque -> should not report conflict
    resource1.hardware_interface = "hardware_interface::EffortJointInterface";
    resource1.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_FALSE(robot.checkForConflict(info_list));

  // Case cartesian pose -> should not report conflict
    resource1.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource1.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_FALSE(robot.checkForConflict(info_list));

  // Case cartesian velocity -> should not report conflict
    resource1.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
    resource1.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1));
    ASSERT_FALSE(robot.checkForConflict(info_list));

// Test dual claims on single arm  ////////////////////////////////////////////////////////////////////////////////////

    // Case joint torque and joint torque -> should report conflict
    resource1.hardware_interface = "hardware_interface::EffortJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "hardware_interface::EffortJointInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case joint torque and joint Position -> should not find conflict
    resource1.hardware_interface = "hardware_interface::EffortJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "hardware_interface::PositionJointInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_FALSE(robot.checkForConflict(info_list));

    // Case joint torque and joint Velocity -> should not find conflict
    resource1.hardware_interface = "hardware_interface::EffortJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "hardware_interface::VelocityJointInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_FALSE(robot.checkForConflict(info_list));

    // Case joint torque and cartesian pose -> should not find conflict
    resource1.hardware_interface = "hardware_interface::EffortJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_FALSE(robot.checkForConflict(info_list));

    // Case joint torque and cartesian velocity -> should not find conflict
    resource1.hardware_interface = "hardware_interface::EffortJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_FALSE(robot.checkForConflict(info_list));


    // Case joint position and joint position -> should report conflict
    resource1.hardware_interface = "hardware_interface::PositionJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "hardware_interface::PositionJointInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case joint position and joint velocity -> should report conflict
    resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "hardware_interface::PositionJointInterface";
    resource2.resources = resources_set;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case joint position and cartesian pose -> should report conflict
    resource1.hardware_interface = "hardware_interface::PositionJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource2.resources = resources_set_cart;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case joint position and cartesian velocity -> should report conflict
    resource1.hardware_interface = "hardware_interface::PositionJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
    resource2.resources = resources_set_cart;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case joint velocity and cartesian velocity -> should report conflict
    resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
    resource2.resources = resources_set_cart;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case joint velocity and cartesian Pose -> should report conflict
    resource1.hardware_interface = "hardware_interface::VelocityJointInterface";
    resource1.resources = resources_set;
    resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource2.resources = resources_set_cart;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));

    // Case cartesian velocity and cartesian pose -> should report conflict
    resource1.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
    resource1.resources = resources_set_cart;
    resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource2.resources = resources_set_cart;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_TRUE(robot.checkForConflict(info_list));


// Test triple claim on single arm ////////////////////////////////////////////////////////
    //  Case 3 command interfaces -> should report conflict
    resource1.hardware_interface = "franka_hw::FrankaVelocityCartesianInterface";
    resource1.resources = resources_set_cart;
    hardware_interface::ControllerInfo info = newInfo("controller1", "dummyControllerClass", resource1, resource1);
    info.claimed_resources.push_back(resource1);
    info_list.clear();
    info_list.push_back(info);
    ASSERT_TRUE(robot.checkForConflict(info_list));


// Test claims on two arms ////////////////////////////////////////////////////////////////////
    // Case 2 arms cartesian pose -> should not report conflict
    resource1.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource1.resources = resources_set_cart;
    resource2.hardware_interface = "franka_hw::FrankaPoseCartesianInterface";
    resource2.resources = resource_set_cart_other_arm;
    info_list.clear();
    info_list.push_back(newInfo("controller1", "dummyControllerClass", resource1, resource2));
    ASSERT_FALSE(robot.checkForConflict(info_list));
}

TEST(FrankaHWTests, interfacesOk) {
  ros::NodeHandle nh;
  std::string arm_id("franka");
  std::vector<std::string> joint_names(7);
  for (size_t i = 0; i < 7; ++i) {
    joint_names[i] = arm_id + "_joint" + std::to_string(i);
  }
  franka_hw::FrankaHW robot(joint_names, nullptr, 30.0, arm_id, nh);

  hardware_interface::JointStateInterface* js_interface = robot.get<hardware_interface::JointStateInterface>();
  hardware_interface::PositionJointInterface* pj_interface = robot.get<>(hardware_interface::PositionJointInterface);
  hardware_interface::VelocityJointInterface* vj_interface = robot.get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* ej_interface = robot.get<hardware_interface::EffortJointInterface>();
  franka_hw::FrankaPoseCartesianInterface* fpc_interface = robot.get<franka_hw::FrankaPoseCartesianInterface>();
  franka_hw::FrankaVelocityCartesianInterface* fvc_interface = robot.get<franka_hw::FrankaVelocityCartesianInterface>();
  franka_hw::FrankaJointStateInterface* fjs_interface = robot.get<franka_hw::FrankaJointStateInterface>();
  franka_hw::FrankaCartesianStateInterface* fcs_interface = robot.get<franka_hw::FrankaCartesianStateInterface>();

  ASSERT_TRUE(js_interface != NULL);
  ASSERT_TRUE(pj_interface != NULL);
  ASSERT_TRUE(vj_interface != NULL);
  ASSERT_TRUE(ej_interface != NULL);
  ASSERT_TRUE(fpc_interface != NULL);
  ASSERT_TRUE(fvc_interface != NULL);
  ASSERT_TRUE(fjs_interface != NULL);
  ASSERT_TRUE(fcs_interface != NULL);

// Test handle of franka_hw cartesian interfaces
  franka_hw::FrankaCartesianPoseHandle fpc_handle = fpc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 16> command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  fpc_handle.setCommand(command);
  ASSERT_TRUE(command == fpc_handle.getCommand());

  franka_hw::FrankaCartesianPoseHandle vpc_handle = vpc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 6> command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  vpc_handle.setCommand(command);
  ASSERT_TRUE(command == vpc_handle.getCommand());
}

TEST(FrankaHWTests, prepareSwitchOk) {
  // TODO
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "FrankaHWTestNode");
  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
