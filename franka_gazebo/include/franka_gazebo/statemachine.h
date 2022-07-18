#pragma once

#include <franka/robot_state.h>
#include <ros/ros.h>
#include <boost_sml/sml.hpp>

namespace franka_gazebo {
// States
struct Idle {};
struct Move {};
struct UserStopped {};

// Events
struct ErrorRecovery {};
struct UserStop {
  bool pressed;
};
struct StartControl {};
struct StopControl {};

// Guards
auto isPressed = [](const UserStop& event) { return event.pressed; };
auto isReleased = [](const UserStop& event) { return not event.pressed; };

// Actions
auto start = [](franka::RobotState& state) { state.robot_mode = franka::RobotMode::kMove; };
auto idle = [](franka::RobotState& state) { state.robot_mode = franka::RobotMode::kIdle; };
auto stop = [](franka::RobotState& state) {
  ROS_WARN("E-Stop pressed, stopping robot");
  state.robot_mode = franka::RobotMode::kUserStopped;
  state.q_d = state.q;
  state.dq_d = {0};
  state.ddq_d = {0};
};

struct StateMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        // clang-format off
       *state<Idle>        + event<StartControl>         / start = state<Move>,
        state<Idle>        + event<UserStop>[isPressed]  / stop  = state<UserStopped>,
        state<Idle>        + event<ErrorRecovery>        / start = state<Move>,
        state<Move>        + event<StopControl>          / idle  = state<Idle>,
        state<Move>        + event<UserStop>[isPressed]  / stop  = state<UserStopped>,
        state<UserStopped> + event<UserStop>[isReleased] / idle  = state<Idle>
        // clang-format on
    );
  }
};
}  // namespace franka_gazebo
