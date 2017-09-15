// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <franka_control/SetCartesianImpedance.h>
#include <franka_control/SetEEFrame.h>
#include <franka_control/SetForceTorqueCollisionBehavior.h>
#include <franka_control/SetFullCollisionBehavior.h>
#include <franka_control/SetJointImpedance.h>
#include <franka_control/SetKFrame.h>
#include <franka_control/SetLoad.h>

namespace franka_control {

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

void setCartesianImpedance(franka::Robot& robot,
                           const SetCartesianImpedance::Request& req,
                           SetCartesianImpedance::Response& res);
void setJointImpedance(franka::Robot& robot,
                       const SetJointImpedance::Request& req,
                       SetJointImpedance::Response& res);
void setEEFrame(franka::Robot& robot, const SetEEFrame::Request& req, SetEEFrame::Response& res);
void setKFrame(franka::Robot& robot, const SetKFrame::Request& req, SetKFrame::Response& res);
void setForceTorqueCollisionBehavior(franka::Robot& robot,
                                     const SetForceTorqueCollisionBehavior::Request& req,
                                     SetForceTorqueCollisionBehavior::Response& res);
void setFullCollisionBehavior(franka::Robot& robot,
                              const SetFullCollisionBehavior::Request& req,
                              SetFullCollisionBehavior::Response& res);
void setLoad(franka::Robot& robot, const SetLoad::Request& req, SetLoad::Response& res);

}  // namespace franka_control
