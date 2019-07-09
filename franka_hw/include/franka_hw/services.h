// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <franka_msgs/SetCartesianImpedance.h>
#include <franka_msgs/SetEEFrame.h>
#include <franka_msgs/SetForceTorqueCollisionBehavior.h>
#include <franka_msgs/SetFullCollisionBehavior.h>
#include <franka_msgs/SetJointImpedance.h>
#include <franka_msgs/SetKFrame.h>
#include <franka_msgs/SetLoad.h>

#include <vector>

namespace franka_hw {

template <typename T>
ros::ServiceServer advertiseService(
    ros::NodeHandle& node_handle,
    const std::string& name,
    std::function<void(typename T::Request&, typename T::Response&)> handler) {
  return node_handle.advertiseService<typename T::Request, typename T::Response>(
      name, [name, handler](typename T::Request& request, typename T::Response& response) {
        try {
          handler(request, response);
          response.success = true;
          ROS_DEBUG_STREAM(name << " succeeded.");
        } catch (const franka::Exception& ex) {
          ROS_ERROR_STREAM(name << " failed: " << ex.what());
          response.success = false;
          response.error = ex.what();
        }
        return true;
      });
}

class ServiceContainer {
 public:
  template <typename T, typename... TArgs>
  ServiceContainer& advertiseService(TArgs&&... args) {
    ros::ServiceServer server = franka_hw::advertiseService<T>(std::forward<TArgs>(args)...);
    services_.push_back(server);
    return *this;
  }

 private:
  std::vector<ros::ServiceServer> services_;
};

void setupServices(franka::Robot& robot, ros::NodeHandle& node_handle, ServiceContainer& services);

// void setupRecoveryActionServer()

void setCartesianImpedance(franka::Robot& robot,
                           const franka_msgs::SetCartesianImpedance::Request& req,
                           franka_msgs::SetCartesianImpedance::Response& res);
void setJointImpedance(franka::Robot& robot,
                       const franka_msgs::SetJointImpedance::Request& req,
                       franka_msgs::SetJointImpedance::Response& res);
void setEEFrame(franka::Robot& robot,
                const franka_msgs::SetEEFrame::Request& req,
                franka_msgs::SetEEFrame::Response& res);
void setKFrame(franka::Robot& robot,
               const franka_msgs::SetKFrame::Request& req,
               franka_msgs::SetKFrame::Response& res);
void setForceTorqueCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::SetForceTorqueCollisionBehavior::Request& req,
    franka_msgs::SetForceTorqueCollisionBehavior::Response& res);
void setFullCollisionBehavior(franka::Robot& robot,
                              const franka_msgs::SetFullCollisionBehavior::Request& req,
                              franka_msgs::SetFullCollisionBehavior::Response& res);
void setLoad(franka::Robot& robot,
             const franka_msgs::SetLoad::Request& req,
             franka_msgs::SetLoad::Response& res);

}  // namespace franka_hw
