#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
//cpp源文件相当于”内容“，每个函数具体的实现方法
namespace mobile_base_hardware{


hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=  // 如果根据urdf（info）的初始化不成功，就返回失败
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    //初始化刚刚我们在private里面设置的一些变量
    left_motor_id_ = std::stoi(info_.hardware_parameters["left_motor_id"]);
    right_motor_id_ = std::stoi(info_.hardware_parameters["right_motor_id"]);
    port_ = info_.hardware_parameters["dynamixel_port"];

    driver_ = std::make_shared<XL330Driver>(port_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure //硬件初始化
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (driver_ -> init() !=0){ //在硬件驱动文件中的init，如果成功就是0,不成功就不是0
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate //启动马达模式
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;


    left_position_ = 0.0;   //激活的时候把这些值都设为0一下
    left_velocity_ = 0.0;
    left_velocity_cmd_ = 0.0;
    right_position_ = 0.0;
    right_velocity_ = 0.0;
    right_velocity_cmd_ = 0.0;


    driver_ -> activateWithVelocityMode(left_motor_id_);
    driver_ -> activateWithVelocityMode(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate //关闭马达模式
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_ -> deactivate(left_motor_id_);
    driver_ -> deactivate(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type MobileBaseHardwareInterface::read //读取
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    double left_vel = driver_ -> getVelocityRadianPerSec(left_motor_id_);
    double right_vel = driver_ -> getVelocityRadianPerSec(right_motor_id_);
    left_velocity_ = left_vel;
    right_velocity_ = right_vel;
    left_position_ = left_position_ + left_vel * period.seconds();
    right_position_ = right_position_ + right_vel * period.seconds();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write //写入
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    driver_->setTargetVelocityRadianPerSec(left_motor_id_, left_velocity_cmd_);
    driver_->setTargetVelocityRadianPerSec(right_motor_id_, right_velocity_cmd_);
    return hardware_interface::return_type::OK;

}

std::vector<hardware_interface::StateInterface> 
  MobileBaseHardwareInterface::export_state_interfaces() //用来广播变量到 状态接口 上的（反向）
  {
      std::vector<hardware_interface::StateInterface> state_interfaces;

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_left_wheel_joint", "position", &left_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_left_wheel_joint", "velocity", &left_velocity_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_right_wheel_joint", "position", &right_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_right_wheel_joint", "velocity", &right_velocity_));

      return state_interfaces;
  }

std::vector<hardware_interface::CommandInterface> 
  MobileBaseHardwareInterface::export_command_interfaces() //用来广播变量到 命令接口 上的（正向）
  {
      std::vector<hardware_interface::CommandInterface> command_interfaces;

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          "base_left_wheel_joint", "velocity", &left_velocity_cmd_));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          "base_right_wheel_joint", "velocity", &right_velocity_cmd_));

      return command_interfaces;
  }


} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface,hardware_interface::SystemInterface) 
//把我们的硬件接口作为一个插件，参数是hpp开头里面（命名空间::class名字 ， 父类名字）