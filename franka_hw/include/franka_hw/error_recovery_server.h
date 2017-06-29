#include <actionlib/server/simple_action_server.h>
#include <franka/robot.h>
#include <franka_hw/ErrorRecoveryAction.h>
#include <ros/ros.h>

namespace franka_hw {

class ErrorRecoveryServer {
 public:
  ErrorRecoveryServer(ros::NodeHandle node_handle,
                      std::string name,
                      franka::Robot* robot);

  ~ErrorRecoveryServer() {}
  void executeRecovery(const franka_hw::ErrorRecoveryGoalConstPtr& goal);

 private:
  actionlib::SimpleActionServer<franka_hw::ErrorRecoveryAction> action_server_;
  franka_hw::ErrorRecoveryFeedback feedback_;
  franka_hw::ErrorRecoveryResult result_;
  franka::Robot* robot_;
  std::string action_name_;
};

}  // namespace franka_hw
