
#include <franka/control_types.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <functional>

namespace franka_hw {
void FrankaHW::runJointPosition(std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::JointValues {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::JointValues(position_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runJointVelocity(std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::JointVelocities {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::JointVelocities(velocity_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runCartesianPose(std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::CartesianPose {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          franka_hw::FrankaCartesianPoseHandle pose_handle(
              franka_pose_cartesian_interface_.getHandle(
                  arm_id_ + std::string("_cartesian")));
          return franka::CartesianPose(pose_handle.getCommand());
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runCartesianVelocity(std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state)
          -> franka::CartesianVelocities {
            if (controller_running_flag_) {
              robot_state_ = robot_state;
              ros_callback();
              franka_hw::FrankaCartesianVelocityHandle velocity_handle(
                  franka_velocity_cartesian_interface_.getHandle(
                      arm_id_ + std::string("_cartesian")));
              return franka::CartesianVelocities(velocity_handle.getCommand());
            } else {
              return franka::Stop;
            }
          });
}

void FrankaHW::runJointTorqueControl(std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::Torques {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(effort_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runTorqueControlWithJointPositionMotionGenerator(
    std::function<void(void)> ros_callback) {
  // TODO: Unnecessary??
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::JointValues {
        if (controller_running_flag_) {
          return franka::JointValues(position_joint_command_);
        } else {
          return franka::Stop;
        }
      },
      [&](const franka::RobotState& robot_state) -> franka::Torques {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(effort_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runTorqueControlWithJointVelocityMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::JointVelocities {
        if (controller_running_flag_) {
          return franka::JointVelocities(velocity_joint_command_);
        } else {
          return franka::Stop;
        }
      },
      [&](const franka::RobotState& robot_state) -> franka::Torques {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(effort_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runTorqueControlWithCartesianPoseMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state) -> franka::CartesianPose {
        if (controller_running_flag_) {
          franka_hw::FrankaCartesianPoseHandle pose_handle(
              franka_pose_cartesian_interface_.getHandle(
                  arm_id_ + std::string("_cartesian")));
          return franka::CartesianPose(pose_handle.getCommand());
        } else {
          return franka::Stop;
        }
      },
      [&](const franka::RobotState& robot_state) -> franka::Torques {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(effort_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

void FrankaHW::runTorqueControlWithCartesianVelocityMotionGenerator(
    std::function<void(void)> ros_callback) {
  robot_->control(
      [&](const franka::RobotState& robot_state)
          -> franka::CartesianVelocities {
            if (controller_running_flag_) {
              franka_hw::FrankaCartesianVelocityHandle velocity_handle(
                  franka_velocity_cartesian_interface_.getHandle(
                      arm_id_ + std::string("_cartesian")));
              return franka::CartesianVelocities(velocity_handle.getCommand());
            } else {
              return franka::Stop;
            }
          },
      [&](const franka::RobotState& robot_state) -> franka::Torques {
        if (controller_running_flag_) {
          robot_state_ = robot_state;
          ros_callback();
          return franka::Torques(effort_joint_command_);
        } else {
          return franka::Stop;
        }
      });
}

}  // namespace franka_hw
