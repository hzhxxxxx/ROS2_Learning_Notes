# ROS 2 Humble + Gazebo Harmonic Bridge 配置指南

## 问题背景

用户发现 Gazebo 官方文档提到旧版的 "ignition" 前缀已改为 "gz" 前缀，但在检查本地 Gazebo 话题类型时，仍然显示 `ignition` 前缀。经过分析发现系统同时安装了两个版本的 Gazebo：
- Gazebo Garden (7.9.0) - 使用 `gz` 命令
- Ignition Gazebo Fortress (6.17.0) - 使用 `ign` 命令

## 解决方案概述

采用非默认配置方案：**ROS 2 Humble + Gazebo Harmonic LTS**，而不是默认的 ROS 2 Humble + Gazebo Fortress 配对。

## 详细实施步骤

### 1. 系统清理与重装

#### 1.1 完全移除旧版本
```bash
# 停止所有相关进程
sudo pkill -f gazebo
sudo pkill -f ign

# 移除所有 Gazebo 和 Ignition 相关包
sudo apt purge -y 'gz-*' 'ignition-*' 'libignition-*' 'libgz-*'
sudo apt purge -y ros-humble-gazebo* ros-humble-ros-gz* ros-humble-ros-ign*

# 移除旧的 ROS 2 安装（因为默认包含 Gazebo Fortress）
sudo apt purge -y 'ros-humble-*'

# 清理残留配置
sudo apt autoremove -y
sudo apt autoclean
```

#### 1.2 重新安装 ROS 2 Humble（仅基础包）
```bash
# 安装基础 ROS 2 Humble（不包含 Gazebo）
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-rviz2 ros-humble-tf2-tools \
    ros-humble-joint-state-publisher-gui
```

### 2. 独立安装 Gazebo Harmonic LTS

```bash
# 安装 Gazebo Harmonic LTS
sudo apt install -y gz-harmonic
```

**验证安装：**
```bash
gz sim --version
# 输出: Gazebo Sim, version 8.9.0
```

### 3. 安装 ROS-Gazebo 桥接包

```bash
# 安装 ROS 2 Humble 与 Gazebo Harmonic 的桥接包
sudo apt install -y ros-humble-ros-gzharmonic-*
```

**安装的关键包：**
- `ros-humble-ros-gzharmonic-bridge`
- `ros-humble-ros-gzharmonic-sim`
- `ros-humble-ros-gzharmonic-interfaces`
- `ros-humble-ros-gzharmonic-image`

### 4. 环境配置

#### 4.1 恢复 .bashrc 配置
```bash
# 编辑 ~/.bashrc 添加：
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### 4.2 重新构建工作空间
```bash
cd ~/ros2_ws
rm -rf install/ build/ log/
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 5. 配置 ros_gz_bridge

#### 5.1 创建 YAML 配置文件
文件路径：`/home/hzh/ros2_ws/src/my_robot_bringup/config/gazebo_bridge.yaml`

```yaml
# Clock synchronization (Gazebo to ROS)
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "ignition.msgs.Clock"
  direction: GZ_TO_ROS

# Joint states from Gazebo to ROS
- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/empty/model/my_robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "ignition.msgs.Model"
  direction: GZ_TO_ROS

# Transform messages from Gazebo to ROS
- ros_topic_name: "/tf"
  gz_topic_name: "/model/my_robot/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "ignition.msgs.Pose_V"
  direction: GZ_TO_ROS

# Odometry data from Gazebo to ROS
- ros_topic_name: "/odom"
  gz_topic_name: "/model/my_robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "ignition.msgs.Odometry"
  direction: GZ_TO_ROS
```

#### 5.2 修改启动文件
文件路径：`/home/hzh/ros2_ws/src/my_robot_bringup/launch/my_robot_gazebo.launch.xml`

```xml
<!-- 使用 YAML 配置文件启动 ros_gz_bridge -->
<let name="bridge_config" value="$(find-pkg-share my_robot_bringup)/config/gazebo_bridge.yaml" />
<node pkg="ros_gz_bridge" exec="parameter_bridge" args="--ros-args -p config_file:=$(var bridge_config)" />
```

### 6. 安装 RQT 工具

```bash
# 安装完整的 RQT 插件集
sudo apt install -y ros-humble-rqt-common-plugins ros-humble-rqt-graph \
    ros-humble-rqt-console ros-humble-rqt-bag ros-humble-rqt-publisher \
    ros-humble-rqt-service-caller ros-humble-rqt-msg ros-humble-rqt-srv \
    ros-humble-rqt-action ros-humble-rqt-reconfigure ros-humble-rqt-py-console \
    ros-humble-rqt-shell
```

## 验证与测试

### 1. 检查 Gazebo 话题
```bash
gz topic -l
```

### 2. 检查消息类型
```bash
gz topic -i -t /clock
# 输出: ignition.msgs.Clock
```

### 3. 测试 ROS-Gazebo 桥接
```bash
# 启动完整系统
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

# 在另一个终端检查 ROS 话题
ros2 topic list
```

### 4. 使用 RQT 工具
```bash
# 启动节点图查看器
rqt_graph

# 启动话题监控
rqt_topic
```

## 重要发现

1. **消息类型兼容性**: 即使在 Gazebo Harmonic (8.9.0) 中，为了向后兼容，消息类型仍然使用 `ignition.msgs.*` 前缀。

2. **命令前缀更新**: 
   - 旧版: `ign` 命令
   - 新版: `gz` 命令

3. **ROS 包命名更新**:
   - 旧版: `ros_ign_*` 包
   - 新版: `ros_gz_*` 包

## 配置优势

- **最新功能**: 使用 Gazebo Harmonic LTS 的最新特性
- **长期支持**: Gazebo Harmonic 是 LTS 版本
- **清洁环境**: 移除了版本冲突问题
- **标准化**: 使用 YAML 配置文件管理桥接

## 参考资源

- [Gazebo 官方文档 - ROS 2 启动](https://gazebosim.org/docs/latest/ros2_launch_gazebo/)
- [Gazebo 官方文档 - ROS 2 集成](https://gazebosim.org/docs/latest/ros2_integration/)
- [ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)

## 常用命令

```bash
# 检查 Gazebo 版本
gz sim --version

# 检查 ROS 包
ros2 pkg list | grep gz

# 启动系统
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

# 查看话题
ros2 topic list
gz topic -l

# 启动调试工具
rqt_graph
```

---

**创建日期**: 2025-10-05  
**ROS 版本**: ROS 2 Humble  
**Gazebo 版本**: Gazebo Harmonic LTS (8.9.0)  
**系统**: Ubuntu 22.04