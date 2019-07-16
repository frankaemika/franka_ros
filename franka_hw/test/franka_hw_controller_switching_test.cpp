// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <list>
#include <random>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>

using std::string;
using std::set;
using hardware_interface::InterfaceResources;
using hardware_interface::ControllerInfo;

std::string arm_id("panda");
std::array<std::string, 7> joint_names = {
    arm_id + "_joint1", arm_id + "_joint2", arm_id + "_joint3", arm_id + "_joint4",
    arm_id + "_joint5", arm_id + "_joint6", arm_id + "_joint7"};

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

class ControllerConflict
    : public ::testing::TestWithParam<std::list<hardware_interface::ControllerInfo>> {
 public:
  ControllerConflict() : robot_(std::make_unique<FrankaHW>()) {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle root_nh;
    EXPECT_TRUE(robot_->initParameters(root_nh, private_nh));
    robot_->initROSInterfaces(private_nh);
  }
  bool callCheckForConflict(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->checkForConflict(info_list);
  }
  bool callPrepareSwitch(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->prepareSwitch(info_list, info_list);
  }

 private:
  std::unique_ptr<FrankaHW> robot_;
};

class NoControllerConflict
    : public ::testing::TestWithParam<std::list<hardware_interface::ControllerInfo>> {
 public:
  NoControllerConflict() : robot_(std::make_unique<FrankaHW>()) {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle root_nh;
    EXPECT_TRUE(robot_->initParameters(root_nh, private_nh));
    robot_->initROSInterfaces(private_nh);
    robot_->setupParameterCallbacks(private_nh);
  }
  bool callCheckForConflict(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->checkForConflict(info_list);
  }
  bool callPrepareSwitch(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->prepareSwitch(info_list, info_list);
  }

 private:
  std::unique_ptr<FrankaHW> robot_;
};

string arm_id2("panda2");
string jp_iface_str("hardware_interface::PositionJointInterface");
string jv_iface_str("hardware_interface::VelocityJointInterface");
string jt_iface_str("hardware_interface::EffortJointInterface");
string cv_iface_str("franka_hw::FrankaVelocityCartesianInterface");
string cp_iface_str("franka_hw::FrankaPoseCartesianInterface");
string unknown_iface_str("hardware_interface::UnknownInterface");
string name_str("some_controller");
string type_str("SomeControllerClass");
set<string> joints_set = {joint_names[0], joint_names[1], joint_names[2], joint_names[3],
                          joint_names[4], joint_names[5], joint_names[6]};
set<string> cartesian_set = {arm_id + "_robot"};
set<string> cartesian_arm2_set = {arm_id2 + "_robot"};
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
                        ControllerConflict,
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
                        NoControllerConflict,
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

TEST_P(ControllerConflict, ConflictsForIncompatibleControllers) {
  EXPECT_TRUE(callCheckForConflict(GetParam()));
}

TEST_P(NoControllerConflict, DoesNotConflictForCompatibleControllers) {
  EXPECT_FALSE(callCheckForConflict(GetParam()));
}

TEST_P(NoControllerConflict, CanPrepareSwitchForCompatibleControllers) {
  EXPECT_TRUE(callPrepareSwitch(GetParam()));
}

}  // namespace franka_hw
