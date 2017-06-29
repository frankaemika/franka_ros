#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/error_recovery_server.h>
#include <functional>
#include <string>

namespace franka_hw {

ErrorRecoveryServer::ErrorRecoveryServer(ros::NodeHandle node_handle,
                                         std::string name,
                                         franka::Robot* robot)
    : action_name_(name),
      action_server_(node_handle,
                     name,
                     std::bind(&ErrorRecoveryServer::executeRecovery,
                               this,
                               std::placeholders::_1),
                     false),
      robot_(robot) {
  action_server_.start();
}

void ErrorRecoveryServer::executeRecovery(
    const franka_hw::ErrorRecoveryGoalConstPtr& goal) {
  feedback_.feedback = "ok";
  result_.result = "none";
  bool success = false;

  if (!robot_) {
    ROS_ERROR("Error recovery server has no robot!");
    result_.result = "robot missing";
    feedback_.feedback = "no robot connected";
    action_server_.publishFeedback(feedback_);
    action_server_.setAborted(result_);
    return;
  } else {
    ROS_INFO("Starting error recovery");
    feedback_.feedback = "starting error recovery";
    action_server_.publishFeedback(feedback_);

    if (action_server_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      feedback_.feedback = "preempted";
      action_server_.publishFeedback(feedback_);
      action_server_.setPreempted();
      return;
    } else {
      try {
        robot_->automaticErrorRecovery();
        success = true;
        feedback_.feedback = "success";
        result_.result = "success";
      } catch (franka::CommandException e) {
        ROS_ERROR_STREAM("" << e.what());
        result_.result = e.what();
        feedback_.feedback = e.what();
      } catch (...) {
        feedback_.feedback = "unknown exception thrown";
        result_.result = "unknown exception thrown";
      }
      action_server_.publishFeedback(feedback_);

      if (success) {
        action_server_.setSucceeded(result_);
      } else {
        action_server_.setAborted(result_);
      }
      return;
    }
  }
}

}  // franka_hw
