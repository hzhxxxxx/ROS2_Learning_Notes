#!/usr/bin/env python3
"""
使用末端位姿控制机器人移动
参考: pymoveit2/examples/ex_pose_goal.py
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from transforms3d.euler import euler2quat


def main():
    rclpy.init()

    # 创建节点
    node = Node("ex_pose_goal")

    # 创建回调组 - 允许并行执行
    callback_group = ReentrantCallbackGroup()

    # 创建 MoveIt 2 接口
    moveit2 = MoveIt2(
        node=node,
        joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        base_link_name='base_link',
        end_effector_name='tool_link',
        group_name='arm',
        callback_group=callback_group,
    )

    # 启动节点的后台线程
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # 设置速度和加速度缩放因子 (0.0 到 1.0)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    #---------------------------------------------------------------

    """
    # 用关节角度作为目标

    # 目标关节角度 (对应 C++ 的 { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 })
    joint_positions = [1.5, 0.5, 0.0, 1.5, 0.0, -0.7]

    # 移动到目标关节配置
    node.get_logger().info(f"Moving to joint positions: {joint_positions}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    node.get_logger().info("Movement completed!")
    """

    #---------------------------------------------------------------

    """
    # 用末端位姿（在基坐标系）做目标

    # 因为moveit里面用的不是rpx, rpy这种欧拉角，而是四元数，所以需要一个人工转化的过程
    # Python 使用 transforms3d.euler2quat 来转换
    # 注意：euler2quat 返回 (w, x, y, z)，需要转换为 [x, y, z, w]
    # 注意：转换四元数这块，其实用/home/hzh/ros2_ws/src/turtle_nav_demo/turtle_nav_demo/my_nav_demo.py这个程序里面的tf_transformations这个库的方法更简洁！
    quat_wxyz = euler2quat(3.14, 0.0, 0.0)  # roll, pitch, yaw (弧度)
    quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]

    # 设置目标位姿
    position = [0.0, -0.7, 0.4]  # x, y, z (在 base_link 坐标系下)

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=False  # 不使用笛卡尔规划
    )
    moveit2.wait_until_executed()

    node.get_logger().info("Movement completed!")
    """

    #---------------------------------------------------------------

    # 笛卡尔直线路径
    # 对应 C++ 的 computeCartesianPath

    # 注意：pymoveit2 不支持完整的 computeCartesianPath API
    # 这里使用 cartesian=True 模式来实现笛卡尔直线运动

    # 先移动到一个初始位姿（对应 C++ 的第68-99行）
    quat_wxyz = euler2quat(3.14, 0.0, 0.0)
    quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]

    initial_position = [0.0, -0.7, 0.4]

    node.get_logger().info("Moving to initial position...")
    moveit2.move_to_pose(
        position=initial_position,
        quat_xyzw=quat_xyzw,
        cartesian=False
    )
    moveit2.wait_until_executed()

    # waypoint 1: 向下移动 0.2m (对应 C++ pose1)
    waypoint1_position = [0.0, -0.7, 0.2]  # z: 0.4 - 0.2 = 0.2

    node.get_logger().info("Moving to waypoint 1 (down 0.2m) using Cartesian path...")
    moveit2.move_to_pose(
        position=waypoint1_position,
        quat_xyzw=quat_xyzw,
        cartesian=True,  # 使用笛卡尔直线规划
        cartesian_max_step=0.01  # 对应 C++ 的 0.01 步长
    )
    moveit2.wait_until_executed()

    # waypoint 2: 向 x 方向移动 0.2m (对应 C++ pose2)
    waypoint2_position = [0.2, -0.7, 0.2]  # x: 0.0 + 0.2 = 0.2

    node.get_logger().info("Moving to waypoint 2 (x +0.2m) using Cartesian path...")
    moveit2.move_to_pose(
        position=waypoint2_position,
        quat_xyzw=quat_xyzw,
        cartesian=True,
        cartesian_max_step=0.01
    )
    moveit2.wait_until_executed()

    # waypoint 3: 向 x 负方向移动 0.2m，向上移动 0.2m (对应 C++ pose3)
    waypoint3_position = [0.0, -0.7, 0.4]  # x: 0.2 - 0.2 = 0.0, z: 0.2 + 0.2 = 0.4

    node.get_logger().info("Moving to waypoint 3 (x -0.2m, z +0.2m) using Cartesian path...")
    moveit2.move_to_pose(
        position=waypoint3_position,
        quat_xyzw=quat_xyzw,
        cartesian=True,
        cartesian_max_step=0.01
    )
    moveit2.wait_until_executed()

    node.get_logger().info("Cartesian path completed!")

    # 参数说明：
    # cartesian=True: 使用笛卡尔直线规划（对应 C++ 的 computeCartesianPath）
    # cartesian_max_step=0.01: 步长 0.01m（对应 C++ 的第二个参数）
    #
    # 关于 jump_threshold（C++ 的第三个参数 0.0）:
    # pymoveit2 内部处理，通过 moveit2.cartesian_jump_threshold 设置
    # 默认值通常为 0.0（禁用跳跃检测）

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
