#pragma once

#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <franka_hw/SetCartesianImpedance.h>
#include <franka_hw/SetEEFrame.h>
#include <franka_hw/SetForceTorqueCollisionBehavior.h>
#include <franka_hw/SetFullCollisionBehavior.h>
#include <franka_hw/SetJointImpedance.h>
#include <franka_hw/SetKFrame.h>
#include <franka_hw/SetLoad.h>
#include <franka_hw/SetTimeScalingFactor.h>

namespace franka_hw {
namespace services {

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
void setTimeScalingFactor(franka::Robot& robot,
                          const SetTimeScalingFactor::Request& req,
                          SetTimeScalingFactor::Response& res);

}  // namespace services
}  // namespace franka_hw
