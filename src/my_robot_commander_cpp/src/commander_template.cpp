#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/string.hpp>
#include <my_robot_interfaces/msg/joint_command.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using String = example_interfaces::msg::String;
using JointCommand = my_robot_interfaces::msg::JointCommand;
using PoseCommand = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander //这个 类 不是 节点，而是包含节点
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_ ->setMaxVelocityScalingFactor(1.0);
        arm_ ->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        //创建这个订阅者，话题名字，10,回调函数
        open_gripper_sub_ = node_->create_subscription<Bool>(
                "open_gripper", 10 , std::bind(&Commander::OpenGripperCallback, this, _1));

        named_target_sub_ = node_->create_subscription<String>(
                "named_target", 10, std::bind(&Commander::NamedTargetCallback, this, _1));

        joint_target_sub_ = node_->create_subscription<JointCommand>(
                "joint_target", 10, std::bind(&Commander::JointTargetCallback, this, _1));

        pose_target_sub_ = node_->create_subscription<PoseCommand>(
                "pose_target", 10, std::bind(&Commander::PoseTargetCallback, this, _1));
    }

    // 用设定好的点位名字，作为目标：
    void goToNamedTarget(const std::string &name)
    {
        //[1]
        arm_->setStartStateToCurrentState(); //设置初始状态为当前状态
        arm_->setNamedTarget(name); // 设置目标点位
        planAndExecute(arm_);
    }

    // 用关节参数作为目标：
    void goToJointTarget(const std::vector<double> &joints)
    {
        //[1]
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }


    // 用末端位姿（在基坐标系（也是手动选的））做目标
    //这个roll ， pitch ,yaw ,其实就是rpx，rpy，rpz
    //包含一个bool值，用来选择是不是直线运动
    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
    {
        //[1]
        // 因为moveit里面用的不是rpx ，rpy这种欧拉角，而是四元数，所以需要一个人工转化的过程
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw); //我们这里写入欧拉角，这三行会自动帮我转成四元数给moveit
        q = q.normalize();
    
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link"; //header的时间戳会自动填充，但是需要一个基坐标（接下来的末端位姿都是在这个基坐标下的位姿！）
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX(); //欧拉角部分，对应上面的转换成四元数部分
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        //如果只是普通的运动
        if (!cartesian_path){
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        //如果是笛卡尔直线运动
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory; //定义一个”轨迹“

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            //计算笛卡尔路径（根据要求的路径点集waypoints，计算出目标笛卡尔轨迹trajectory）
            //第二个参数0.01是步长，越小越平滑但是计算点越多
            //第三个参数jump_threshold指的是关节跳跃阀值，是一个安全参数,用来检测关节角度的突变。
            //当=0.0的时候，就是禁用跳跃检测 (允许任意大的关节角度变化)，更灵活但是更不平滑稳定
            //当>0的时候，(比如 2.0) = 如果相邻两点之间任何关节角度变化超过这个值(弧度),就认为是"跳跃",规划失败
            //这个最后的返回值fraction，是规划的成功率，1就是100%都成功了

            if (fraction == 1){
                arm_->execute(trajectory);
            }
        }
        
    }


    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open"); // 直接用名字就行
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }

private:

    //其实就是原来的plan和execute部分,传入的数据就是“group”
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); 
        
        if (success) {
            interface->execute(plan);
        }   
    
    }

    void OpenGripperCallback(const Bool &msg)
    {
        if (msg.data){
            openGripper();
        }
        else{
            closeGripper();
        }
    }

    void NamedTargetCallback(const String &msg)
    {
        goToNamedTarget(msg.data);
    }

    void JointTargetCallback(const JointCommand &msg)
    {
        std::vector<double> joints(msg.joint_positions.begin(), msg.joint_positions.end());
        goToJointTarget(joints);
    }

    void PoseTargetCallback(const PoseCommand &msg)
    {
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<String>::SharedPtr named_target_sub_;
    rclcpp::Subscription<JointCommand>::SharedPtr joint_target_sub_;
    rclcpp::Subscription<PoseCommand>::SharedPtr pose_target_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}