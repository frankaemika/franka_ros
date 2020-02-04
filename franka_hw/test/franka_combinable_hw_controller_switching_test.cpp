// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <list>
#include <memory>
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
#include <franka_hw/franka_combinable_hw.h>

extern std::string arm_id;
extern std::array<std::string, 7> joint_names;

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

class CombinableControllerConflict
    : public ::testing::TestWithParam<std::list<hardware_interface::ControllerInfo>> {
 public:
  CombinableControllerConflict() : robot_(std::make_unique<franka_hw::FrankaCombinableHW>()) {
    ros::NodeHandle root_nh;
    ros::NodeHandle robot_hw_nh("~");
    robot_->initParameters(root_nh, robot_hw_nh);
    robot_->initROSInterfaces(robot_hw_nh);
    robot_->setupParameterCallbacks(robot_hw_nh);
  }
  bool callCheckForConflict(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->checkForConflict(info_list);
  }
  bool callPrepareSwitch(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->prepareSwitch(info_list, info_list);
  }

 private:
  std::unique_ptr<franka_hw::FrankaCombinableHW> robot_;
};

class CombinableNoControllerConflict
    : public ::testing::TestWithParam<std::list<hardware_interface::ControllerInfo>> {
 public:
  CombinableNoControllerConflict() : robot_(std::make_unique<franka_hw::FrankaCombinableHW>()) {
    ros::NodeHandle root_nh;
    ros::NodeHandle robot_hw_nh("~");
    robot_->initParameters(root_nh, robot_hw_nh);
    robot_->initROSInterfaces(robot_hw_nh);
    robot_->setupParameterCallbacks(robot_hw_nh);
  }
  bool callCheckForConflict(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->checkForConflict(info_list);
  }
  bool callPrepareSwitch(const std::list<hardware_interface::ControllerInfo> info_list) {
    return robot_->prepareSwitch(info_list, info_list);
  }

 private:
  std::unique_ptr<franka_hw::FrankaCombinableHW> robot_;
};

std::string arm_id2("panda2");
std::string jp_iface_str("hardware_interface::PositionJointInterface");
std::string jv_iface_str("hardware_interface::VelocityJointInterface");
std::string jt_iface_str("hardware_interface::EffortJointInterface");
std::string cv_iface_str("franka_hw::FrankaVelocityCartesianInterface");
std::string cp_iface_str("franka_hw::FrankaPoseCartesianInterface");
std::string unknown_iface_str("hardware_interface::UnknownInterface");
std::string name_str("some_controller");
std::string type_str("SomeControllerClass");
std::set<std::string> joints_set = {joint_names[0], joint_names[1], joint_names[2], joint_names[3],
                                    joint_names[4], joint_names[5], joint_names[6]};
std::set<std::string> cartesian_set = {arm_id + "_robot"};
std::set<std::string> cartesian_arm2_set = {arm_id2 + "_robot"};
std::set<std::string> no_id_set = {"joint1"};
hardware_interface::InterfaceResources no_id_res(jp_iface_str, no_id_set);
hardware_interface::InterfaceResources unknown_iface_res(unknown_iface_str, joints_set);
hardware_interface::InterfaceResources jp_res(jp_iface_str, joints_set);
hardware_interface::InterfaceResources jv_res(jv_iface_str, joints_set);
hardware_interface::InterfaceResources jt_res(jt_iface_str, joints_set);
hardware_interface::InterfaceResources cv_res(cv_iface_str, cartesian_set);
hardware_interface::InterfaceResources cp_res(cp_iface_str, cartesian_set);
hardware_interface::InterfaceResources cp_arm2_res(cp_iface_str, cartesian_arm2_set);
hardware_interface::ControllerInfo cp_arm2_info = newInfo(name_str, type_str, cp_arm2_res);
hardware_interface::ControllerInfo no_id_info = newInfo(name_str, type_str, no_id_res);
hardware_interface::ControllerInfo unknown_iface_info =
    newInfo(name_str, type_str, unknown_iface_res);
hardware_interface::ControllerInfo jt_jt_info = newInfo(name_str, type_str, jt_res, jt_res);
hardware_interface::ControllerInfo jp_jp_info = newInfo(name_str, type_str, jp_res, jp_res);
hardware_interface::ControllerInfo jp_jv_info = newInfo(name_str, type_str, jp_res, jv_res);
hardware_interface::ControllerInfo jp_cv_info = newInfo(name_str, type_str, jp_res, cv_res);
hardware_interface::ControllerInfo jp_cp_info = newInfo(name_str, type_str, jp_res, cp_res);
hardware_interface::ControllerInfo jv_jv_info = newInfo(name_str, type_str, jv_res, jv_res);
hardware_interface::ControllerInfo jv_cv_info = newInfo(name_str, type_str, jv_res, cv_res);
hardware_interface::ControllerInfo jv_cp_info = newInfo(name_str, type_str, jv_res, cp_res);
hardware_interface::ControllerInfo cv_cv_info = newInfo(name_str, type_str, cv_res, cv_res);
hardware_interface::ControllerInfo cv_cp_info = newInfo(name_str, type_str, cv_res, cp_res);
hardware_interface::ControllerInfo cp_cp_info = newInfo(name_str, type_str, cp_res, cp_res);
hardware_interface::ControllerInfo jp_jp_jp_info =
    newInfo(name_str, type_str, jp_res, jp_res, jp_res);
hardware_interface::ControllerInfo jp_info = newInfo(name_str, type_str, jp_res);
hardware_interface::ControllerInfo jv_info = newInfo(name_str, type_str, jv_res);
hardware_interface::ControllerInfo jt_info = newInfo(name_str, type_str, jt_res);
hardware_interface::ControllerInfo cv_info = newInfo(name_str, type_str, cv_res);
hardware_interface::ControllerInfo cp_info = newInfo(name_str, type_str, cp_res);
hardware_interface::ControllerInfo jt_jp_info = newInfo(name_str, type_str, jt_res, jp_res);
hardware_interface::ControllerInfo jt_jv_info = newInfo(name_str, type_str, jt_res, jv_res);
hardware_interface::ControllerInfo jt_cv_info = newInfo(name_str, type_str, jt_res, cv_res);
hardware_interface::ControllerInfo jt_cp_info = newInfo(name_str, type_str, jt_res, cp_res);

INSTANTIATE_TEST_CASE_P(
    combinableNonAdmissibleRequests,
    CombinableControllerConflict,
    ::testing::Values(std::list<hardware_interface::ControllerInfo>{no_id_info},
                      std::list<hardware_interface::ControllerInfo>{unknown_iface_info},
                      std::list<hardware_interface::ControllerInfo>{jp_info},
                      std::list<hardware_interface::ControllerInfo>{jv_info},
                      std::list<hardware_interface::ControllerInfo>{cv_info},
                      std::list<hardware_interface::ControllerInfo>{cp_info},
                      std::list<hardware_interface::ControllerInfo>{jt_jp_info},
                      std::list<hardware_interface::ControllerInfo>{jt_jv_info},
                      std::list<hardware_interface::ControllerInfo>{jt_jt_info},
                      std::list<hardware_interface::ControllerInfo>{jp_jp_info},
                      std::list<hardware_interface::ControllerInfo>{jp_jv_info},
                      std::list<hardware_interface::ControllerInfo>{jp_cv_info},
                      std::list<hardware_interface::ControllerInfo>{jp_cp_info},
                      std::list<hardware_interface::ControllerInfo>{jv_jv_info},
                      std::list<hardware_interface::ControllerInfo>{jv_cv_info},
                      std::list<hardware_interface::ControllerInfo>{jv_cp_info},
                      std::list<hardware_interface::ControllerInfo>{cv_cv_info},
                      std::list<hardware_interface::ControllerInfo>{cv_cp_info},
                      std::list<hardware_interface::ControllerInfo>{cp_cp_info},
                      std::list<hardware_interface::ControllerInfo>{jp_jp_jp_info},
                      std::list<hardware_interface::ControllerInfo>{jt_cv_info},
                      std::list<hardware_interface::ControllerInfo>{jt_cp_info}));

INSTANTIATE_TEST_CASE_P(combinableAdmissibleRequests,
                        CombinableNoControllerConflict,
                        ::testing::Values(std::list<hardware_interface::ControllerInfo>{jt_info},
                                          std::list<hardware_interface::ControllerInfo>{
                                              jt_info, cp_arm2_info}));

TEST(FrankaCombinableHWTests, CanInitROSInterfaces) {
  franka_hw::FrankaCombinableHW hw;
  ros::NodeHandle root_nh;
  ros::NodeHandle robot_hw_nh("~");
  EXPECT_TRUE(hw.initParameters(root_nh, robot_hw_nh));
  EXPECT_NO_THROW(hw.initROSInterfaces(robot_hw_nh));
}

TEST_P(CombinableControllerConflict, ConflictsForIncompatibleControllers) {
  EXPECT_TRUE(callCheckForConflict(GetParam()));
}

TEST_P(CombinableNoControllerConflict, DoesNotConflictForCompatibleControllers) {
  EXPECT_FALSE(callCheckForConflict(GetParam()));
}

TEST_P(CombinableNoControllerConflict, CanPrepareSwitchForCompatibleControllers) {
  EXPECT_TRUE(callPrepareSwitch(GetParam()));
}
