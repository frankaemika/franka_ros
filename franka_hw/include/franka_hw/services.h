// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <vector>

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

namespace franka_hw {

/**
 * Advertises a service that acts according to the handler function which is used in the service
 * callback.
 *
 * @param[in] node_handle The NodeHandle in the namespace at which to advertise the service.
 * @param[in] name The name of the service.
 * @param[in] handler The callback method for the service.
 * @return The service server.
 */
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

/**
 * This class serves as container that gathers all possible service interfaces to a libfranka robot
 * instance.
 */
class ServiceContainer {
 public:
  /**
   * Advertises and adds a service to the container class.
   *
   * @return A reference to the service container to allow a fluent API.
   */
  template <typename T, typename... TArgs>
  ServiceContainer& advertiseService(TArgs&&... args) {
    ros::ServiceServer server = franka_hw::advertiseService<T>(std::forward<TArgs>(args)...);
    services_.push_back(server);
    return *this;
  }

 private:
  std::vector<ros::ServiceServer> services_;
};

/**
 * Sets up all services relevant for a libfranka robot inside a service container.
 *
 * @param[in] robot The libfranka robot for which to set up services interfaces.
 * @param[in] node_handle The NodeHandle in the namespace at which to advertise the services.
 * @param[in] services The container to store the service servers.
 */
void setupServices(franka::Robot& robot, ros::NodeHandle& node_handle, ServiceContainer& services);

/**
 * Callback for the service interface to franka::robot::setCartesianImpedance.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setCartesianImpedance(franka::Robot& robot,
                           const franka_msgs::SetCartesianImpedance::Request& req,
                           franka_msgs::SetCartesianImpedance::Response& res);

/**
 * Callback for the service interface to franka::robot::setJointImpedance.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setJointImpedance(franka::Robot& robot,
                       const franka_msgs::SetJointImpedance::Request& req,
                       franka_msgs::SetJointImpedance::Response& res);

/**
 * Callback for the service interface to franka::robot::setEEFrame.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setEEFrame(franka::Robot& robot,
                const franka_msgs::SetEEFrame::Request& req,
                franka_msgs::SetEEFrame::Response& res);

/**
 * Callback for the service interface to franka::robot::setKFrame.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setKFrame(franka::Robot& robot,
               const franka_msgs::SetKFrame::Request& req,
               franka_msgs::SetKFrame::Response& res);

/**
 * Callback for the service interface to franka::robot::setForceTorqueCollisionBehavior.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setForceTorqueCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::SetForceTorqueCollisionBehavior::Request& req,
    franka_msgs::SetForceTorqueCollisionBehavior::Response& res);

/**
 * Callback for the service interface to franka::robot::setFullCollisionBehavior.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setFullCollisionBehavior(franka::Robot& robot,
                              const franka_msgs::SetFullCollisionBehavior::Request& req,
                              franka_msgs::SetFullCollisionBehavior::Response& res);

/**
 * Callback for the service interface to franka::robot::setLoad.
 *
 * @param[in] robot The libfranka robot for which to set up the service.
 * @param[in] req The service request.
 * @param[out] res The service response.
 */
void setLoad(franka::Robot& robot,
             const franka_msgs::SetLoad::Request& req,
             franka_msgs::SetLoad::Response& res);

}  // namespace franka_hw
