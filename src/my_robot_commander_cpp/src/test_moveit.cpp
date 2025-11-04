#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    // 启动一个多线程的动作，让节点一直保持启动状态，并且不会阻塞接下来的代码
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // 创建 MoveGroupInterface
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    // 设置速度和加速度缩放因子 (0.0 到 1.0)
    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);

    //---------------------------------------------------------------

    /*
    // 用设定好的点位名字，作为目标：

    //[1]
    arm.setStartStateToCurrentState(); //设置初始状态为当前状态
    arm.setNamedTarget("pose_1"); // 设置目标点位

    //[2]
    // plan部分
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1)) == moveit::core::MoveItErrorCode::SUCCESS; // 检查一下plan是否成功

    //[3]
    // execute部分
    if (success1) {
        arm.execute(plan1);
    }
    */

    //---------------------------------------------------------------

    /*
    // 用关节参数作为目标：
    //[1]
    // 各关节参数向量
    std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    //[2]
    // plan部分
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1)) == moveit::core::MoveItErrorCode::SUCCESS; // 检查一下plan是否成功

    //[3]
    // execute部分
    if (success1) {
        arm.execute(plan1);
    }
    */

    //---------------------------------------------------------------

    // 用末端位姿（在基坐标系（也是手动选的））做目标


    //[1]
    // 因为moveit里面用的不是rpx ，rpy这种欧拉角，而是四元数，所以需要一个人工转化的过程
    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0); //我们这里写入欧拉角，这三行会自动帮我转成四元数给moveit
    q = q.normalize();
    
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link"; //header的时间戳会自动填充，但是需要一个基坐标（接下来的末端位姿都是在这个基坐标下的位姿！）
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX(); //欧拉角部分，对应上面的转换成四元数部分
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    //[2]
    // plan部分
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1)) == moveit::core::MoveItErrorCode::SUCCESS; // 检查一下plan是否成功

    //[3]
    // execute部分
    if (success1) {
        arm.execute(plan1);
    }

    //---------------------------------------------------------------
    
    // 笛卡尔直线路径

    std::vector<geometry_msgs::msg::Pose> waypoints; //这里做的其实是定义一个直线路径点集
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += -0.2;
    waypoints.push_back(pose1); // 因为我们这里就一个规划点，所以这个点集只有一个点
    
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.x += 0.2;
    waypoints.push_back(pose2);
    
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.x += -0.2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);

    moveit_msgs::msg::RobotTrajectory trajectory; //定义一个”轨迹“

    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    //计算笛卡尔路径（根据要求的路径点集waypoints，计算出目标笛卡尔轨迹trajectory）
    //第二个参数0.01是步长，越小越平滑但是计算点越多
    //第三个参数jump_threshold指的是关节跳跃阀值，是一个安全参数,用来检测关节角度的突变。
        //当=0.0的时候，就是禁用跳跃检测 (允许任意大的关节角度变化)，更灵活但是更不平滑稳定
        //当>0的时候，(比如 2.0) = 如果相邻两点之间任何关节角度变化超过这个值(弧度),就认为是"跳跃",规划失败
    //这个最后的返回值fraction，是规划的成功率，1就是100%都成功了

    if (fraction == 1){
        arm.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join(); //等待刚刚的多线程终止

    return 0;
}