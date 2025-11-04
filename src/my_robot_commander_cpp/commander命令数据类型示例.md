## 直接名字点位：
ros2 topic pub --once /named_target example_interfaces/msg/String "{data: 'home'}"

## 关节目标：
ros2 topic pub --once /joint_target my_robot_interfaces/msg/JointCommand "{joint_positions: [1.5, 0.5, 0.0, 1.5, 0.0, -0.7]}"

## 末端位姿目标（普通运动）：
ros2 topic pub --once /pose_target my_robot_interfaces/msg/PoseCommand "{x: 0.0, y: -0.7, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"

##　末端位姿目标（笛卡尔直线运动）：
ros2 topic pub --once /pose_target my_robot_interfaces/msg/PoseCommand "{x: 0.0, y: -0.7, z: 0.6, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: true}"

##　夹爪控制：
# 打开夹爪
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: true}"  

# 关闭夹爪
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: false}"
