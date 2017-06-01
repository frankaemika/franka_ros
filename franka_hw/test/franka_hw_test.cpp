#include <gtest/gtest.h>
#include <array>
#include <list>
#include <random>
#include <set>
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

using std::string;
using std::set;
using hardware_interface::InterfaceResources;
using hardware_interface::ControllerInfo;


namespace franka_hw {

hardware_interface::ControllerInfo newInfo(
    const std::string& name,
    const std::string& type,
    const hardware_interface::InterfaceResources& resource1) {
  hardware_interface::ControllerInfo info;
  info.claimed_resources.clear();
  info.name = name;
  info.type = type;
  info.claimed_resources.push_back(resource1);
  return info;
}

hardware_interface::ControllerInfo newInfo(
    const std::string& name,
    const std::string& type,
    const hardware_interface::InterfaceResources& resource1,
    const hardware_interface::InterfaceResources& resource2) {
  hardware_interface::ControllerInfo info = newInfo(name, type, resource1);
  info.claimed_resources.push_back(resource2);
  return info;
}

hardware_interface::ControllerInfo newInfo(
    const std::string& name,
    const std::string& type,
    const hardware_interface::InterfaceResources& resource1,
    const hardware_interface::InterfaceResources& resource2,
    const hardware_interface::InterfaceResources& resource3) {
  hardware_interface::ControllerInfo info = newInfo(name, type, resource1, resource2);
  info.claimed_resources.push_back(resource3);
  return info;
}

FrankaHW* createMockRobot() {
    ros::NodeHandle nh;
    std::string arm_id("franka_emika");
    std::vector<std::string> joint_names(7);
    for (size_t i = 0; i < 7; ++i) {
      joint_names[i] = arm_id + "_joint" + std::to_string(i + 1);
    }
    return new FrankaHW(joint_names, nullptr, 30.0, arm_id, nh);
}

class findConflictsTestFixture :
        public ::testing::TestWithParam<std::list<hardware_interface::ControllerInfo> > {
public:
 findConflictsTestFixture() : robot_(franka_hw::createMockRobot()) {}
 bool callCheckForConflict(const std::list<hardware_interface::ControllerInfo> info_list) {
     return robot_->checkForConflict(info_list);
 }
 bool callPrepareSwitch(const std::list<hardware_interface::ControllerInfo> info_list) {
     return robot_->prepareSwitch(info_list, info_list);
 }
 private:
 std::unique_ptr<franka_hw::FrankaHW>  robot_;
};

class findAdmissibleAndPrepareSwitchTestFixture : public
        ::testing::TestWithParam<std::list<hardware_interface::ControllerInfo> > {
public:
 findAdmissibleAndPrepareSwitchTestFixture() : robot_(franka_hw::createMockRobot()) {}
 bool callCheckForConflict(const std::list<hardware_interface::ControllerInfo> info_list) {
     return robot_->checkForConflict(info_list);
 }
 bool callPrepareSwitch(const std::list<hardware_interface::ControllerInfo> info_list) {
     return robot_->prepareSwitch(info_list, info_list);
 }
 private:
 std::unique_ptr<franka_hw::FrankaHW>  robot_;
};

string arm_id("franka_emika");
string arm_id2("franka_emika2");
std::vector<std::string> joint_names  = {arm_id + "_joint1",
        arm_id + "_joint2",
        arm_id + "_joint3",
        arm_id + "_joint4",
        arm_id + "_joint5",
        arm_id + "_joint6",
        arm_id + "_joint7"};
string jp_iface_str("hardware_interface::PositionJointInterface");
string jv_iface_str("hardware_interface::VelocityJointInterface");
string jt_iface_str("hardware_interface::EffortJointInterface");
string cv_iface_str("franka_hw::FrankaVelocityCartesianInterface");
string cp_iface_str("franka_hw::FrankaPoseCartesianInterface");
string unknown_iface_str("hardware_interface::UnknownInterface");
string name_str("some_controller");
string type_str("SomeControllerClass");

set<string> joints_set = {joint_names[0],
                          joint_names[1],
                          joint_names[2],
                          joint_names[3],
                          joint_names[4],
                          joint_names[5],
                          joint_names[6]};
set<string> cartesian_set = {arm_id + "_cartesian"};
set<string> cartesian_arm2_set = {arm_id2 + "_cartesian"};
set<string> no_id_set = {"joint1"};

InterfaceResources no_id_res(jp_iface_str, no_id_set);
InterfaceResources unknown_iface_res(unknown_iface_str, joints_set);
InterfaceResources jp_res(jp_iface_str, joints_set);
InterfaceResources jv_res(jv_iface_str, joints_set);
InterfaceResources jt_res(jt_iface_str, joints_set);
InterfaceResources cv_res(cv_iface_str, cartesian_set);
InterfaceResources cp_res(cp_iface_str, cartesian_set);
InterfaceResources cp_arm2_res(cp_iface_str, cartesian_arm2_set);

ControllerInfo cp_arm2_info = newInfo(name_str, type_str, cp_arm2_res);
ControllerInfo no_id_info = newInfo(name_str, type_str, no_id_res);
ControllerInfo unknown_iface_info = newInfo(name_str, type_str, unknown_iface_res);
ControllerInfo jt_jt_info = newInfo(name_str, type_str, jt_res, jt_res);
ControllerInfo jp_jp_info = newInfo(name_str, type_str, jp_res, jp_res);
ControllerInfo jp_jv_info = newInfo(name_str, type_str, jp_res, jv_res);
ControllerInfo jp_cv_info = newInfo(name_str, type_str, jp_res, cv_res);
ControllerInfo jp_cp_info = newInfo(name_str, type_str, jp_res, cp_res);
ControllerInfo jv_jv_info = newInfo(name_str, type_str, jv_res, jv_res);
ControllerInfo jv_cv_info = newInfo(name_str, type_str, jv_res, cv_res);
ControllerInfo jv_cp_info = newInfo(name_str, type_str, jv_res, cp_res);
ControllerInfo cv_cv_info = newInfo(name_str, type_str, cv_res, cv_res);
ControllerInfo cv_cp_info = newInfo(name_str, type_str, cv_res, cp_res);
ControllerInfo cp_cp_info = newInfo(name_str, type_str, cp_res, cp_res);
ControllerInfo jp_jp_jp_info = newInfo(name_str, type_str, jp_res, jp_res, jp_res);
ControllerInfo jp_info = newInfo(name_str, type_str, jp_res);
ControllerInfo jv_info = newInfo(name_str, type_str, jv_res);
ControllerInfo jt_info = newInfo(name_str, type_str, jt_res);
ControllerInfo cv_info = newInfo(name_str, type_str, cv_res);
ControllerInfo cp_info = newInfo(name_str, type_str, cp_res);
ControllerInfo jt_jp_info = newInfo(name_str, type_str, jt_res, jp_res);
ControllerInfo jt_jv_info = newInfo(name_str, type_str, jt_res, jv_res);
ControllerInfo jt_cv_info = newInfo(name_str, type_str, jt_res, cv_res);
ControllerInfo jt_cp_info = newInfo(name_str, type_str, jt_res, cp_res);

INSTANTIATE_TEST_CASE_P(nonAdmissibleRequests,
                        findConflictsTestFixture,
                        ::testing::Values(std::list<ControllerInfo>{no_id_info},
                                          std::list<ControllerInfo>{unknown_iface_info},
                                          std::list<ControllerInfo>{jt_jt_info},
                                          std::list<ControllerInfo>{jp_jp_info},
                                          std::list<ControllerInfo>{jp_jv_info},
                                          std::list<ControllerInfo>{jp_cv_info},
                                          std::list<ControllerInfo>{jp_cp_info},
                                          std::list<ControllerInfo>{jv_jv_info},
                                          std::list<ControllerInfo>{jv_cv_info},
                                          std::list<ControllerInfo>{jv_cp_info},
                                          std::list<ControllerInfo>{cv_cv_info},
                                          std::list<ControllerInfo>{cv_cp_info},
                                          std::list<ControllerInfo>{cp_cp_info},
                                          std::list<ControllerInfo>{jp_jp_jp_info}));

INSTANTIATE_TEST_CASE_P(admissibleRequests,
                        findAdmissibleAndPrepareSwitchTestFixture,
                        ::testing::Values(std::list<ControllerInfo>{jp_info},
                                          std::list<ControllerInfo>{jv_info},
                                          std::list<ControllerInfo>{jt_info},
                                          std::list<ControllerInfo>{cv_info},
                                          std::list<ControllerInfo>{cp_info},
                                          std::list<ControllerInfo>{jt_jp_info},
                                          std::list<ControllerInfo>{jt_jv_info},
                                          std::list<ControllerInfo>{jt_cv_info},
                                          std::list<ControllerInfo>{jt_cp_info},
                                          std::list<ControllerInfo>{cp_info, cp_arm2_info}));

TEST_P(findConflictsTestFixture, findConflictsOk) {
  EXPECT_TRUE(callCheckForConflict(GetParam()));
}

TEST_P(findAdmissibleAndPrepareSwitchTestFixture, findAdmissibleOk) {
  EXPECT_FALSE(callCheckForConflict(GetParam()));
}

TEST_P(findAdmissibleAndPrepareSwitchTestFixture, prepareSwitchOk) {
    EXPECT_TRUE(callPrepareSwitch(GetParam()));
}

TEST(FrankaHWTests, interfacesOk) {
  std::unique_ptr<franka_hw::FrankaHW> robotptr(createMockRobot());
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

  ASSERT_TRUE(js_interface != NULL);
  ASSERT_TRUE(pj_interface != NULL);
  ASSERT_TRUE(vj_interface != NULL);
  ASSERT_TRUE(ej_interface != NULL);
  ASSERT_TRUE(fpc_interface != NULL);
  ASSERT_TRUE(fvc_interface != NULL);
  ASSERT_TRUE(fjs_interface != NULL);
  ASSERT_TRUE(fcs_interface != NULL);

  franka_hw::FrankaCartesianPoseHandle fpc_handle =
      fpc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 16> pose_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0};
  fpc_handle.setCommand(pose_command);
  EXPECT_TRUE(pose_command == fpc_handle.getCommand());

  franka_hw::FrankaCartesianVelocityHandle fvc_handle =
      fvc_interface->getHandle(arm_id + "_cartesian");
  std::array<double, 6> vel_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  fvc_handle.setCommand(vel_command);
  EXPECT_TRUE(vel_command == fvc_handle.getCommand());

  EXPECT_NO_THROW(fcs_interface->getHandle(arm_id + "_cartesian"));

  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NO_THROW(fjs_interface->getHandle(joint_names[i]));
  }
}

TEST(FrankaHWTests, jointLimitInterfacesOk) {
  std::unique_ptr<franka_hw::FrankaHW> robot_ptr(createMockRobot());

  hardware_interface::PositionJointInterface* pj_interface =
      robot_ptr->get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface* vj_interface =
      robot_ptr->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* ej_interface =
      robot_ptr->get<hardware_interface::EffortJointInterface>();
  ASSERT_TRUE(pj_interface != NULL);
  ASSERT_TRUE(vj_interface != NULL);
  ASSERT_TRUE(ej_interface != NULL);

  urdf::Model urdf_model;
  ros::NodeHandle nh;
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
  robot_ptr->enforceLimits(ros::Duration(0.001));
  for (size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_TRUE(
        position_handles[i].getCommand() <= joint_limits[i].max_position &&
        position_handles[i].getCommand() >= joint_limits[i].min_position);
    EXPECT_TRUE(
        velocity_handles[i].getCommand() <= joint_limits[i].max_velocity &&
        velocity_handles[i].getCommand() >= -joint_limits[i].max_velocity);
    EXPECT_TRUE(effort_handles[i].getCommand() <= joint_limits[i].max_effort &&
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
  robot_ptr->enforceLimits(ros::Duration(0.001));
  for (size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_TRUE(
        position_handles[i].getCommand() <= joint_limits[i].max_position &&
        position_handles[i].getCommand() >= joint_limits[i].min_position);
    EXPECT_TRUE(
        velocity_handles[i].getCommand() <= joint_limits[i].max_velocity &&
        velocity_handles[i].getCommand() >= -joint_limits[i].max_velocity);
    EXPECT_TRUE(effort_handles[i].getCommand() <= joint_limits[i].max_effort &&
                effort_handles[i].getCommand() >= -joint_limits[i].max_effort);
  }
}

}  // namespace franka_hw

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_hw_test_node");
  return RUN_ALL_TESTS();
}
