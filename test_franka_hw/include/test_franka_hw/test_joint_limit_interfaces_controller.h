#include <vector>
#include <string>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.h>
#include <franka_hw/franka_joint_state_interface.h>
#include <urdf/model.h>

namespace test_franka_hw {

class JointLimitTestController : public controller_interface::MultiInterfaceController<
    hardware_interface::PositionJointInterface,
    hardware_interface::VelocityJointInterface,
    hardware_interface::EffortJointInterface,
    franka_hw::FrankaJointStateInterface>
{
public:
  JointLimitTestController();
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

private:
  std::vector<joint_limits_interface::JointLimits> joint_limits_;
  std::vector<joint_limits_interface::SoftJointLimits> soft_joint_limits_;
  std::vector<std::string> joint_names_;
  hardware_interface::PositionJointInterface* position_interface_;
  hardware_interface::VelocityJointInterface* velocity_interface_;
  hardware_interface::EffortJointInterface* effort_interface_;
  franka_hw::FrankaJointStateInterface* franka_joint_state_interface_;
  std::uniform_real_distribution<double> uniform_distribution_;
  std::default_random_engine random_engine_;
  bool phase_ = false;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
};



}  // namespace test_franka_hw
