#include <franka_gazebo/controller_verifier.h>
#include <gtest/gtest.h>

static const std::string effort = "hardware_interface::EffortJointInterface";
static const std::string position = "hardware_interface::PositionJointInterface";
static const std::string velocity = "hardware_interface::VelocityJointInterface";

class ControllerVerifierFixture : public ::testing::Test {
 protected:
  std::unique_ptr<franka_gazebo::ControllerVerifier> verifier;

  virtual void SetUp() {
    std::map<std::string, std::shared_ptr<franka_gazebo::Joint>> joint_map;
    for (int i = 1; i < 8; i++) {
      joint_map.emplace("panda_joint" + std::to_string(i), nullptr);
    }
    for (int i = 1; i < 3; i++) {
      joint_map.emplace("panda_finger_joint" + std::to_string(i), nullptr);
    }
    verifier = std::make_unique<franka_gazebo::ControllerVerifier>(joint_map, "panda");
  }

  virtual void TearDown() {}
};
class ValidArmControllerTest
    : public ControllerVerifierFixture,
      public testing::WithParamInterface<hardware_interface::ControllerInfo> {
 protected:
  void SetUp() override {
    ControllerVerifierFixture::SetUp();
    controller = GetParam();
  }

  hardware_interface::ControllerInfo controller;
};

class NonClaimingArmControllerTest : public ValidArmControllerTest {};
class MalformedArmControllerTest : public ValidArmControllerTest {};
class NonClaimingGripperControllerTest : public ValidArmControllerTest {};
class InvalidGripperControllerTest : public ValidArmControllerTest {};
class ValidGripperControllerTest : public ValidArmControllerTest {};
class MalformedGripperControllerTest : public ValidArmControllerTest {};

hardware_interface::ControllerInfo generate_arm_controller(const std::string& hw_interface,
                                                           int num_joints = 7) {
  hardware_interface::ControllerInfo controller;
  hardware_interface::InterfaceResources resource;
  resource.hardware_interface = hw_interface;
  for (int i = 1; i <= num_joints; i++) {
    resource.resources.emplace("panda_joint" + std::to_string(i));
  }
  controller.claimed_resources.push_back(resource);
  return controller;
}

hardware_interface::ControllerInfo generate_gripper_controller(const std::string& hw_interface,
                                                               int num_joints = 2) {
  hardware_interface::ControllerInfo controller;
  hardware_interface::InterfaceResources resource;
  resource.hardware_interface = hw_interface;
  for (int i = 1; i <= num_joints; i++) {
    resource.resources.emplace("panda_finger_joint" + std::to_string(i));
  }
  controller.claimed_resources.push_back(resource);
  return controller;
}

hardware_interface::ControllerInfo generate_arm_and_gripper_controller(
    const std::string& hw_interface) {
  hardware_interface::ControllerInfo controller;
  hardware_interface::InterfaceResources resource;
  resource.hardware_interface = hw_interface;
  for (int i = 1; i <= 7; i++) {
    resource.resources.emplace("panda_joint" + std::to_string(i));
  }
  for (int i = 1; i <= 2; i++) {
    resource.resources.emplace("panda_finger_joint" + std::to_string(i));
  }
  controller.claimed_resources.push_back(resource);
  return controller;
}

TEST_P(ValidArmControllerTest, IsArmController) {
  EXPECT_TRUE(verifier->isClaimingArmController(controller));
}

TEST_P(ValidArmControllerTest, HasArmJoints) {
  for (const auto& claimed_resources : controller.claimed_resources) {
    EXPECT_TRUE(verifier->areArmJoints(claimed_resources.resources));
  }
}

TEST_P(ValidArmControllerTest, DoesNotHaveFingerJoints) {
  for (const auto& claimed_resources : controller.claimed_resources) {
    EXPECT_FALSE(verifier->areFingerJoints(claimed_resources.resources));
  }
}

TEST_P(ValidArmControllerTest, IsNotGripperController) {
  EXPECT_FALSE(verifier->isClaimingGripperController(controller));
}

TEST_P(ValidArmControllerTest, IsValidController) {
  EXPECT_TRUE(verifier->isValidController(controller));
}

TEST_P(NonClaimingArmControllerTest, IsValidController) {
  EXPECT_TRUE(verifier->isValidController(controller));
}

TEST_P(NonClaimingArmControllerTest, IsNotArmController) {
  EXPECT_FALSE(verifier->isClaimingArmController(controller));
}

TEST_P(NonClaimingArmControllerTest, IsNotGripperController) {
  EXPECT_FALSE(verifier->isClaimingGripperController(controller));
}

TEST_P(MalformedArmControllerTest, IsValidController) {
  EXPECT_FALSE(verifier->isValidController(controller));
}

TEST_P(MalformedArmControllerTest, IsNotArmController) {
  EXPECT_FALSE(verifier->isClaimingArmController(controller));
}

TEST_P(MalformedArmControllerTest, IsNotGripperController) {
  EXPECT_FALSE(verifier->isClaimingGripperController(controller));
}

TEST_P(ValidGripperControllerTest, IsValidController) {
  EXPECT_TRUE(verifier->isValidController(controller));
}

TEST_P(ValidGripperControllerTest, IsGripperController) {
  EXPECT_TRUE(verifier->isClaimingGripperController(controller));
}

TEST_P(ValidGripperControllerTest, HasFingerJoints) {
  for (const auto& claimed_resources : controller.claimed_resources) {
    EXPECT_TRUE(verifier->areFingerJoints(claimed_resources.resources));
  }
}

TEST_P(ValidGripperControllerTest, DoesNotHaveArmJoints) {
  for (const auto& claimed_resources : controller.claimed_resources) {
    EXPECT_FALSE(verifier->areArmJoints(claimed_resources.resources));
  }
}

TEST_P(NonClaimingGripperControllerTest, DoesNotHaveArmJoints) {
  for (const auto& claimed_resources : controller.claimed_resources) {
    EXPECT_FALSE(verifier->areArmJoints(claimed_resources.resources));
  }
}

TEST_P(NonClaimingGripperControllerTest, IsNotGripperController) {
  EXPECT_FALSE(verifier->isClaimingGripperController(controller));
}

TEST_P(NonClaimingGripperControllerTest, IsNotArmController) {
  EXPECT_FALSE(verifier->isClaimingArmController(controller));
}

TEST_P(NonClaimingGripperControllerTest, IsValidController) {
  EXPECT_TRUE(verifier->isValidController(controller));
}

TEST_P(InvalidGripperControllerTest, DoesNotHaveArmJoints) {
  for (const auto& claimed_resources : controller.claimed_resources) {
    EXPECT_FALSE(verifier->areArmJoints(claimed_resources.resources));
  }
}

TEST_P(InvalidGripperControllerTest, IsNotGripperController) {
  EXPECT_FALSE(verifier->isClaimingGripperController(controller));
}

TEST_P(InvalidGripperControllerTest, IsNotArmController) {
  EXPECT_FALSE(verifier->isClaimingArmController(controller));
}

TEST_P(InvalidGripperControllerTest, IsNotValidController) {
  EXPECT_FALSE(verifier->isValidController(controller));
}

TEST_P(MalformedGripperControllerTest, IsNotValidController) {
  EXPECT_FALSE(verifier->isValidController(controller));
}

TEST_P(MalformedGripperControllerTest, IsNotArmController) {
  EXPECT_FALSE(verifier->isClaimingArmController(controller));
}

TEST_P(MalformedGripperControllerTest, IsNotGripperController) {
  EXPECT_FALSE(verifier->isClaimingGripperController(controller));
}

INSTANTIATE_TEST_CASE_P(ValidArmControllerTest,  // NOLINT(cert-err58-cpp)
                        ValidArmControllerTest,
                        ::testing::Values(generate_arm_controller(effort),
                                          generate_arm_controller(position),
                                          generate_arm_controller(velocity)));

INSTANTIATE_TEST_CASE_P(NonClaimingArmControllerTest,  // NOLINT(cert-err58-cpp)
                        NonClaimingArmControllerTest,
                        ::testing::Values(generate_arm_controller(""),
                                          generate_arm_controller(" "),
                                          generate_arm_controller("safsdafsdafsad"),
                                          generate_arm_and_gripper_controller("asfdasf")));

INSTANTIATE_TEST_CASE_P(MalformedArmControllerTest,  // NOLINT(cert-err58-cpp)
                        MalformedArmControllerTest,
                        ::testing::Values(generate_arm_controller(effort, 6),
                                          generate_arm_controller(position, 8),
                                          generate_arm_controller(velocity, 2),
                                          generate_arm_and_gripper_controller(effort),
                                          generate_arm_and_gripper_controller(position),
                                          generate_arm_and_gripper_controller(velocity)));

INSTANTIATE_TEST_CASE_P(ValidGripperControllerTest,  // NOLINT(cert-err58-cpp)
                        ValidGripperControllerTest,
                        ::testing::Values(generate_gripper_controller(effort)));

INSTANTIATE_TEST_CASE_P(NonClaimingGripperControllerTest,  // NOLINT(cert-err58-cpp)
                        NonClaimingGripperControllerTest,
                        ::testing::Values(generate_gripper_controller(""),
                                          generate_gripper_controller(" "),
                                          generate_gripper_controller("afdasfas")));

INSTANTIATE_TEST_CASE_P(InvalidGripperControllerTest,  // NOLINT(cert-err58-cpp)
                        InvalidGripperControllerTest,
                        ::testing::Values(generate_gripper_controller(velocity),
                                          generate_gripper_controller(position)));

INSTANTIATE_TEST_CASE_P(MalformedGripperControllerTest,  // NOLINT(cert-err58-cpp)
                        MalformedGripperControllerTest,
                        ::testing::Values(generate_gripper_controller(effort, 1),
                                          generate_gripper_controller(effort, 3)));