#include <wsg_50_control/wsg_controller.hpp>
namespace wsg_50_control
{

WSG50HardwareInterface::WSG50HardwareInterface()
: wsg_(), last_goal_position_(0.0),mode_(0)
{
  // Initialisation des membres
}

WSG50HardwareInterface::~WSG50HardwareInterface()
{
  // Destruction des membres
}

hardware_interface::CallbackReturn WSG50HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Remplissage des paramètres du driver
  wsg_.name_ = info.joints[0].name;  // ou info.joints.at(0).name;
  wsg_.ip_ = info.hardware_parameters.at("ip_address");
  wsg_.port_ = std::stoi(info.hardware_parameters.at("port"));
  wsg_.rate_ = std::stod(info.hardware_parameters.at("rate"));
  wsg_.grasping_force_ = std::stod(info.hardware_parameters.at("grasping_force"));


  wsg_.goal_speed_ = 20.0;
  mode_=0;

  std::cout << "Gripper name: " << wsg_.name_ << std::endl;
  std::cout << "Gripper IP: " << wsg_.ip_ << std::endl;
  std::cout << "Gripper port: " << wsg_.port_ << std::endl;
  std::cout << "Gripper rate: " << wsg_.rate_ << std::endl;
  std::cout << "Gripper grasping force: " << wsg_.grasping_force_ << std::endl;


  // if (!wsg_.connect()) {
  //   RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to connect to WSG-50");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  // if (!wsg_.setup()) {
  //   RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to setup WSG-50");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WSG50HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("WSG50HardwareInterface"), "Activating WSG50");

  try {
    if (!wsg_.connect()) {
      RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to connect to WSG-50");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!wsg_.setup()) {
      RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to setup WSG-50");
      return hardware_interface::CallbackReturn::ERROR;
    }

    last_goal_position_ = std::nan("");  // reset pour forcer un premier write()

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Exception in on_activate: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WSG50HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_POSITION, &wsg_.width_);
  state_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_VELOCITY, &wsg_.speed_);
  state_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_EFFORT, &wsg_.force_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WSG50HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_POSITION, &wsg_.goal_width_);
  return command_interfaces;
}

hardware_interface::return_type WSG50HardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Lecture de l'état courant depuis le matériel
  // wsg_.position_ = wsg_.getOpening() / 1000.0; // mm -> m
  // wsg_.speed_ = wsg_.getSpeed() / 1000.0;
  // wsg_.force_ = wsg_.getForce(); // en N

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WSG50HardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  if (std::isnan(last_goal_position_) || std::abs(wsg_.goal_width_ - last_goal_position_) > 1e-4)
  { 
    if (wsg_.goal_width_ < last_goal_position_)
      mode_=1;//grasp
    else
      mode_=2;//release
    if (wsg_.cmd(wsg_.goal_width_, wsg_.goal_speed_,mode_) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to send move command");
      return hardware_interface::return_type::ERROR;
    }
    std::cout << "Sending command to gripper: " << wsg_.goal_width_ << std::endl;
    last_goal_position_ = wsg_.goal_width_;
  }

  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wsg_50_control::WSG50HardwareInterface, hardware_interface::SystemInterface)


}// namespace wsg_50_control
