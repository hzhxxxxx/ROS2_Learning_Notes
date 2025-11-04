#!/usr/bin/env python3
"""
TurtleBot Navigation Demo using Nav2 Simple Commander API
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z): #把设置目标位姿的步骤，写成一个统一的函数
    pose = PoseStamped()
    pose.header.frame_id = 'map' # 地图名字
    pose.header.stamp = navigator.get_clock().now().to_msg() # 时间戳
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z) # 欧拉角转换成四元数 , 并且对于2d来说，其实只有rpz会变化，因为是在xy平面上旋转，这里旋转方向就是右手定则，然后 pi==3.14==180度
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    # --- 【1】Init初始化
    rclpy.init()
    navigator = BasicNavigator() # 导入这个nav的api库

    # --- 【2】设置初始位姿
    initial_pose = create_pose_stamped(navigator, 0.0, 0.0, 0.0) #在2d上面其实定位就是x,y,rpz也就是旋转角度
    navigator.setInitialPose(initial_pose) # 注意这个设置初始位姿这一步，不能重复做两次（相当于在rviz里面在不同位置点两次2d位姿定位），否则机器人会混乱，所以第一次进行完之后，如果还想执行这个程序，可以把这一行给注释掉

    # --- 【3】等待导航完全激活并准备好
    # 前面设置初始位姿，只是发布一个话题消息而已，相当于"提前预约"，这里的激活相当于”开火做饭“，
    # 因此需要提前预约好对的座位，然后厨师开始起锅烧油、反过来的话，可能烧完油发现你要的位置已经被人占了
    navigator.waitUntilNav2Active()
    
    # --- 【4】设置目标位置
    goal_pose1 = create_pose_stamped(navigator, 3.5, 0.0, 1.57)
    goal_pose2 = create_pose_stamped(navigator, 3.5, 1.0, 1.57)
    goal_pose3 = create_pose_stamped(navigator, 0.0, 1.0, 1.57)

    # navigator.goToPose(goal_pose1) # 【5.1】去单独一个点

    waypoints = [goal_pose1, goal_pose2, goal_pose3] 
    navigator.followWaypoints(waypoints) # 【5.2】按照路点走（一点一点走，停）
    # navigator.goThroughPoses(waypoints) # 【5.3】按照路点规划一条完整的流畅的路径（中间不停）

    # 【6】等待到达目标
    # 这个 while 循环有两个作用：1. 实时反馈进度  2. 阻塞等待任务完成（阻塞下一步代码立刻进行）
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # print(f'剩余距离: {feedback.distance_remaining:.2f} 米')

    result = navigator.getResult()
    if 'SUCCEEDED' in str(result):
        print('成功到达目标!')
    else:
        print(f'导航失败: {result}')



    rclpy.shutdown() # 关闭节点


if __name__ == '__main__':
    main()
