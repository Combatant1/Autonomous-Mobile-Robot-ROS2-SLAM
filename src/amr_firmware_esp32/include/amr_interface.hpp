#ifndef AMR_INTERFACE_HPP
#define AMR_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp> //TO INSTEGRATE WITH ROS2 CONTROL
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>
#include <array>


namespace amr_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn; //just to save us from retyping everything out

class AmrInterface : public hardware_interface::SystemInterface
{
public:
  AmrInterface();
  virtual ~AmrInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time , const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort esp_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;

  // The ESP32 firmware drives/senses 4 independent wheels and identifies
  // them on the wire with a numeric id '1'-'4' (see robot_control.ino):
  // 1 = front_right   2 = front_left   3 = rear_right   4 = rear_left
  // wheel_idx_[id-1] gives the index into the *_states_/*_commands_ vectors
  // for that wheel, resolved by joint name in on_init() rather than assumed
  // from URDF declaration order.
  static constexpr int kNumWheels = 4;
  std::array<int, kNumWheels> wheel_idx_{-1, -1, -1, -1};
};
}  // namespace amr_firmware


#endif  // AMR_INTERFACE_HPP