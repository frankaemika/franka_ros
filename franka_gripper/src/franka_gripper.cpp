
#include <franka_gripper/franka_gripper.h>

#include <string>
#include <boost/function.hpp>
#include <functional>

#include <ros/node_handle.h>

#include <franka_gripper/Grasp.h>
#include <franka_gripper/Homing.h>
#include <franka_gripper/Stop.h>
#include <franka_gripper/Move.h>
#include <franka/exception.h>

namespace {
template <typename T1, typename T2>
const boost::function<bool(T1&, T2&)> createServiceCallback(
    std::function<void(T1&, T2&)> handler) {
  return [handler](T1& request, T2& response) -> bool {
    try {
      handler(request, response);
      response.success = true;
    } catch (const franka::CommandException& ex) {
      ROS_ERROR_STREAM("" << ex.what());
      response.success = false;
      response.error = ex.what();
    }
    return true;
  };
}

}  // anonymous namespace


namespace franka_gripper {

using std::placeholders::_1;

GripperServer::GripperServer(const std::string& robot_ip, ros::NodeHandle& node_handle) :
    gripper_(robot_ip),
    move_server_(node_handle.advertiseService("move",
                                              createServiceCallback<Move::Request, Move::Response>(std::bind(&GripperServer::move, this, _1)))),
    homing_server_(node_handle.advertiseService("homing",
                                                createServiceCallback<Homing::Request, Homing::Response>(std::bind(&GripperServer::homing, this)))),
    grasp_server_(node_handle.advertiseService("grasp",
                                              createServiceCallback<Grasp::Request, Grasp::Response>(std::bind(&GripperServer::grasp, this, _1)))),
    stop_server_(node_handle.advertiseService("stop",
                                              createServiceCallback<Stop::Request, Stop::Response>(std::bind(&GripperServer::stop, this)))) {
}

void GripperServer::move(const Move::Request& request) {
        gripper_.move(request.width, request.speed);
}

void GripperServer::homing() {
        gripper_.homing();
}

void GripperServer::stop() {
    gripper_.stop();
}

void GripperServer::grasp(const Grasp::Request& request) {
    gripper_.grasp(request.width, request.speed, request.max_current);
}

}  // namespace franka_gripper

