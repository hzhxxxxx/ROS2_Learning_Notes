#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP 
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP
//这个是头文件保护，相当于给这个文件加上一层一次性进入的门，因为如果重复多次进入会报错
//hpp头文件相当于”目录“，告诉别人有哪些函数
#include "hardware_interface/system_interface.hpp"
//引入官方定义好的接口规范
#include "my_robot_hardware/xl330_driver.hpp"
//注意需要在.vscode中加入这个上级路径（这里include后面是一个文件夹my_robot_hardware）

namespace mobile_base_hardware{

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface //这个public是继承父类，相当于python里面的super()
{  
public: //外部可以访问（覆盖父类方法+新方法都可以放这里）
    //这个public部分基本很通用，所以其它机器人也可以使用这部分
    //Lifecycle node override
    hardware_interface::CallbackReturn  //systeminterface是一个父类同时也是生命节点
        on_configure(const rclcpp_lifecycle::State & previous_state) override; //相当于写了这个函数，生命节点才能离开第一阶段
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override; //相当于写了这个函数，生命节点才能离开第一阶段
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override; //相当于写了这个函数，生命节点才能离开第一阶段

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override; // 读取配置信息urdf等等
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override; // 从电机读取当前状态（位置、速度）
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override; // 把控制命令发给电机
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;  //用来广播 状态接口 和 命令接口 的（把变量跟我们之前的话题名字匹配起来）
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private: //外部不能访问（一般放成员变量）
    std::shared_ptr<XL330Driver> driver_; //指针
    int left_motor_id_;
    int right_motor_id_;
    std::string port_;

    //因为humble旧版本不支持set_state和get_state，所以他是通过设置全局变量，然后设置一个专门的函数用来广播变量。而平常的赋值就是通过赋值到这些全局变量上
    double left_position_ = 0.0;
    double left_velocity_ = 0.0;
    double left_velocity_cmd_ = 0.0;
    double right_position_ = 0.0;
    double right_velocity_ = 0.0;
    double right_velocity_cmd_ = 0.0;

}; //class MobileBaseHardwareInterface

} // namespace mobile_base_hardware

#endif