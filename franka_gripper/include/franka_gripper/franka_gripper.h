#pragma once

#include <cmath>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/node_handle.h>

#include <franka/gripper.h>
#include <franka/exception.h>
#include <franka/gripper_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/HomingAction.h>


namespace {

template <typename T_action, typename T_goal, typename T_result>
void handleErrors(actionlib::SimpleActionServer<T_action>* server, std::function<bool(T_goal&)> handler, T_goal& goal) {
    T_result result;
    try {
        result.success = handler(goal);
        server->setSucceeded(result);
    } catch (const franka::Exception& ex) {
        ROS_ERROR_STREAM("" << ex.what());
        result.success = false;
        result.error = ex.what();
        server->setAborted(result);
    }
}

// usage:
// create handler
// actionlib::SimpleActionServer<T_action> server(node_handle, name, std::bind(&handleErrors, &server , handler, _1), false )


}  // anonymous namespace


namespace franka_gripper {

class GripperServer {
public:

    GripperServer() = delete;

    /**
  * Constructs an instance of GripperServer
  *
  * @param[in] robot_ip The IP address of the robot
  */
    GripperServer(const std::string& robot_ip, ros::NodeHandle &node_handle);

    /**
  * Reads the current gripper state from the hardware
  *
  * @param[in] state A pointer to store the new gripper state
  */
    bool getGripperState(franka::GripperState* state);

private:
    /**
  * Calls the libfranka move service of the gripper
  *
  * @param[in] request A move command with target width and velocity
  */
    bool move(const MoveActionGoalConstPtr &goal);

    /**
  * Calls the libfranka homing service of the gripper
  */
    bool homing(const HomingActionGoalConstPtr& /*goal*/);

    /**
  * Calls the libfranka stop service of the gripper to stop applying force
  */
    bool stop(const StopActionGoalConstPtr& /*goal*/);

    /**
  * Calls the libfranka grasp service of the gripper
  *
  * @param[in] request A grasp command with target width,velocity and max effort
  */
    bool grasp(const GraspActionGoalConstPtr &goal);

    /**
  * A callback function for a control_msgs/GripperCommand action
  *
  * @param[in] goal A gripper action goal
  */
    bool gripperCommandHandler(const control_msgs::GripperCommandGoalConstPtr& goal,
                               franka::GripperState* state);
    void gripperCommandexecuteCallback(const control_msgs::GripperCommandGoalConstPtr& goal);

    franka::Gripper gripper_;
    const double default_speed_{0.1};
    const double newton_to_m_ampere_factor_{15.0};
    const double width_tolerance_{0.005};

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction>
    gripper_command_action_server_;
};

}  // namespace franka_gripper
