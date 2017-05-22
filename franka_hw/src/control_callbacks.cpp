
#include <franka/control_types.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <functional>

namespace franka_hw {
void FrankaHW::runJointPosition(std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::JointValues {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::JointValues(this->position_joint_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runJointVelocity(std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::JointVelocities {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::JointVelocities(this->velocity_joint_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runCartesianPose(std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::CartesianPose {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::CartesianPose(this->pose_cartesian_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runCartesianVelocity(std::function<void(void)> ros_callback) {
  robot_->control([=](const franka::RobotState& robot_state)
                      -> franka::CartesianVelocities {
                        if (this->controller_running_flag_) {
                          this->robot_state_ = robot_state;
                          ros_callback();
                          return franka::CartesianVelocities(
                              this->velocity_cartesian_command_);
                        }
                        return franka::Stop;
                      });
}

void FrankaHW::runJointTorqueControl(std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::Torques {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(this->effort_joint_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runTorqueControlWithJointPositionMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::JointValues {
        if (this->controller_running_flag_) {
          return franka::JointValues(this->position_joint_command_);
        }
        return franka::Stop;
      },
      [=](const franka::RobotState& robot_state) -> franka::Torques {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(this->effort_joint_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runTorqueControlWithJointVelocityMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::JointVelocities {
        if (this->controller_running_flag_) {
          return franka::JointVelocities(this->velocity_joint_command_);
        }
        return franka::Stop;
      },
      [=](const franka::RobotState& robot_state) -> franka::Torques {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(this->effort_joint_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runTorqueControlWithCartesianPoseMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state) -> franka::CartesianPose {
        if (this->controller_running_flag_) {
          return franka::CartesianPose(this->pose_cartesian_command_);
        }
        return franka::Stop;
      },
      [=](const franka::RobotState& robot_state) -> franka::Torques {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(this->effort_joint_command_);
        }
        return franka::Stop;
      });
}

void FrankaHW::runTorqueControlWithCartesianVelocityMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [=](const franka::RobotState& robot_state)
          -> franka::CartesianVelocities {
            if (this->controller_running_flag_) {
              return franka::CartesianVelocities(
                  this->velocity_cartesian_command_);
            }
            return franka::Stop;
          },
      [=](const franka::RobotState& robot_state) -> franka::Torques {
        if (this->controller_running_flag_) {
          this->robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(this->effort_joint_command_);
        }
        return franka::Stop;
      });
}

}  // namespace franka_hw
