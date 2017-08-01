
#include <franka_example_controllers/model_example_controller.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_hw/franka_model_interface.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace franka_example_controllers {

ModelExampleController::ModelExampleController()
    : model_interface_(nullptr), model_handle_(nullptr), rate_trigger_(1.0) {}

bool ModelExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("ModelExampleController: Could not read parameter arm_id");
    return false;
  }
  model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM("ModelExampleController: Error getting model interface from harware");
    return false;
  }

  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface_->getHandle(arm_id_ + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ModelExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
  return true;
}

void ModelExampleController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  if (rate_trigger_()) {
    std::array<double, 49> mass = model_handle_->getMass(
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});
    std::array<double, 7> coriolis = model_handle_->getCoriolis(
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});
    std::array<double, 7> gravity = model_handle_->getGravity(0.0, {{0.0, 0.0, 0.0}});
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("mass :" << mass);
    ROS_INFO_STREAM("coriolis: " << coriolis);
    ROS_INFO_STREAM("gravity :" << gravity);
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelExampleController,
                       controller_interface::ControllerBase)
