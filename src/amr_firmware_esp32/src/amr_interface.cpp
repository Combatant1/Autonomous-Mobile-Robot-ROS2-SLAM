#include "amr_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>


namespace amr_firmware
{
AmrInterface::AmrInterface()
{
}


AmrInterface::~AmrInterface()
{
  if (esp_.IsOpen())
  {
    try
    {
      esp_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AmrInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn AmrInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("AmrInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // reserve() only sets capacity, not size - using .at(i)/[i] afterwards on
  // an empty vector is undefined behaviour. resize() actually allocates
  // info_.joints.size() zero-initialized elements.
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  last_run_ = rclcpp::Clock().now();

  // Resolve which vector index corresponds to which physical wheel by name,
  // rather than assuming a fixed position. Order here matches the numeric
  // wheel id used by the firmware serial protocol (see robot_control.ino):
  //   id '1' = front_right, '2' = front_left, '3' = rear_right, '4' = rear_left
  static constexpr std::array<const char *, kNumWheels> kWheelJointNames{
      "front_right_joint", "front_left_joint", "rear_right_joint", "rear_left_joint"};

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    for (int w = 0; w < kNumWheels; w++)
    {
      if (info_.joints[i].name == kWheelJointNames[w])
      {
        wheel_idx_[w] = static_cast<int>(i);
      }
    }
  }

  for (int w = 0; w < kNumWheels; w++)
  {
    if (wheel_idx_[w] < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("AmrInterface"),
                   "Expected joint '%s' in ros2_control config - check amr_ros2_control.xacro",
                   kWheelJointNames[w]);
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> AmrInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> AmrInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn AmrInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AmrInterface"), "Starting robot hardware ...");

  // Reset commands and states (sized to the actual joint count from on_init)
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  try
  {
    esp_.Open(port_);
    esp_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("AmrInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("AmrInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn AmrInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AmrInterface"), "Stopping robot hardware ...");

  if (esp_.IsOpen())
  {
    try
    {
      esp_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AmrInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AmrInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type AmrInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Interpret the string. Firmware sends one token per wheel, each shaped
  // "<id><sign><value>", where id is '1'-'4' (see robot_control.ino for the
  // front_right/front_left/rear_right/rear_left ordering) and sign is 'p'/'n'.
  if(esp_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    esp_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    while(std::getline(ss, res, ','))
    {
      if (res.size() < 3)
      {
        continue;  // malformed/short token, skip rather than throw
      }

      char id_char = res.at(0);
      if (id_char < '1' || id_char > '4')
      {
        continue;  // unknown wheel id, ignore
      }

      int wheel = id_char - '1';
      int state_idx = wheel_idx_[wheel];
      int multiplier = res.at(1) == 'p' ? 1 : -1;

      velocity_states_.at(state_idx) = multiplier * std::stod(res.substr(2, res.size()));
      position_states_.at(state_idx) += velocity_states_.at(state_idx) * dt;
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AmrInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Implement communication protocol with the ESP32: one token per wheel,
  // "<id><sign><value>," with id '1'-'4' in front_right/front_left/
  // rear_right/rear_left order (matching the firmware).
  std::stringstream message_stream;
  message_stream << std::fixed << std::setprecision(2);

  for (int wheel = 0; wheel < kNumWheels; wheel++)
  {
    double cmd = velocity_commands_.at(wheel_idx_[wheel]);
    char sign = cmd >= 0 ? 'p' : 'n';
    std::string leading_zero = std::abs(cmd) < 10.0 ? "0" : "";

    message_stream << (wheel + 1) << sign << leading_zero << std::abs(cmd) << ",";
  }

  try
  {
    esp_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AmrInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace amr_firmware

PLUGINLIB_EXPORT_CLASS(amr_firmware::AmrInterface, hardware_interface::SystemInterface)