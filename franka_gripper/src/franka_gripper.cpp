#include <franka_gripper/franka_gripper.h>

#include <boost/function.hpp>
#include <cmath>
#include <functional>

#include <franka/exception.h>
#include <franka/gripper_state.h>

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

constexpr double GripperServer::kCommandVelocity;
constexpr double GripperServer::kNewtonToMilliAmpereFactor;
constexpr double GripperServer::kWidthTolerance;

GripperServer::GripperServer(const std::string& robot_ip,
                             ros::NodeHandle& node_handle)
    : gripper_(robot_ip),
      action_server_(node_handle,
                     "gripper_action",
                     std::bind(&GripperServer::executeGripperCommand, this, _1),
                     false),
      move_server_(node_handle.advertiseService(
          "move",
          createServiceCallback<Move::Request, Move::Response>(
              std::bind(&GripperServer::move, this, _1)))),
      homing_server_(node_handle.advertiseService(
          "homing",
          createServiceCallback<Homing::Request, Homing::Response>(
              std::bind(&GripperServer::homing, this)))),
      grasp_server_(node_handle.advertiseService(
          "grasp",
          createServiceCallback<Grasp::Request, Grasp::Response>(
              std::bind(&GripperServer::grasp, this, _1)))),
      stop_server_(node_handle.advertiseService(
          "stop",
          createServiceCallback<Stop::Request, Stop::Response>(
              std::bind(&GripperServer::stop, this)))) {
  action_server_.start();
}

bool GripperServer::getGripperState(franka::GripperState* state) {
  try {
    *state = gripper_.readOnce();
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM(
        "GripperServer: Exception reading gripper state: " << ex.what());
    return false;
  }
  return true;
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

void GripperServer::executeGripperCommand(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  control_msgs::GripperCommandResult command_result;
  try {
    franka::GripperState state = gripper_.readOnce();
    if (goal->command.position > state.max_width ||
        goal->command.position < 0.0) {
      ROS_ERROR_STREAM(
          "GripperServer: Commanding out of range width! max_width = "
          << state.max_width << " command = " << goal->command.position);
      action_server_.setAborted();
      return;
    }
    if (goal->command.position >= state.width) {
      gripper_.move(goal->command.position, kCommandVelocity);
      state = gripper_.readOnce();
      if (std::pow((state.width - goal->command.position), 2) <
          kWidthTolerance) {
        command_result.effort = 0.0;
        command_result.position = state.width;
        command_result.reached_goal =
            true;  // NOLINT [readability-implicit-bool-cast]
        command_result.stalled =
            false;  // NOLINT [readability-implicit-bool-cast]
        action_server_.setSucceeded(command_result);
        return;
      }
      ROS_WARN("GripperServer: Move failed");

    } else {
      ROS_INFO("Action: Grasping");
      gripper_.grasp(goal->command.position, kCommandVelocity,
                     goal->command.max_effort * kNewtonToMilliAmpereFactor);
      ROS_INFO("Action: Grasping finished");
      state = gripper_.readOnce();
      if (state.is_grasped) {
        command_result.effort = 0.0;
        command_result.position = state.width;
        command_result.reached_goal =
            true;  // NOLINT [readability-implicit-bool-cast]
        command_result.stalled =
            false;  // NOLINT [readability-implicit-bool-cast]
        ROS_INFO("Action: Set succeeded");
        action_server_.setSucceeded(command_result);
        return;
      }
      gripper_.stop();
      ROS_WARN("GripperServer: Grasp failed");
    }
  } catch (const franka::CommandException& ex) {
    ROS_ERROR_STREAM(
        "GripperServer: Exception performing grasp: " << ex.what());
  } catch (const franka::NetworkException& ex) {
    ROS_ERROR_STREAM("GripperServer: Exception reading state: " << ex.what());
  } catch (const franka::ProtocolException& ex) {
    ROS_ERROR_STREAM("GripperServer: Exception reading state: " << ex.what());
  }

  action_server_.setAborted();
}

}  // namespace franka_gripper
