# Navigation2 + TurtleBot3 + Gazebo Harmonic 安装指南

## 问题背景

在 **ROS 2 Humble + Gazebo Harmonic LTS** 环境下，尝试安装 TurtleBot3 仿真包时遇到依赖冲突问题。

### 系统环境
- **ROS 版本**: ROS 2 Humble
- **Gazebo 版本**: Gazebo Harmonic LTS (8.9.0)
- **操作系统**: Ubuntu 22.04
- **ROS-Gazebo 桥接**: `ros-humble-ros-gzharmonic-*`

### 遇到的问题

1. **apt 安装冲突**
   ```bash
   sudo apt install ros-humble-turtlebot3-gazebo
   ```
   报错：
   ```
   gz-tools2 : 冲突: gazebo (>= 11.0.0) 但是 11.10.2+dfsg-1 正要被安装
               冲突: gazebo (<= 11.14.0) 但是 11.10.2+dfsg-1 正要被安装
   ```

2. **根本原因**
   - apt 仓库中的 `ros-humble-turtlebot3-gazebo` 包依赖 **Gazebo Classic 11**（旧版）
   - 系统已安装 **Gazebo Harmonic**（新版）
   - 两个版本的 Gazebo **无法共存**

3. **官方文档的误导**
   - [TurtleBot3 官方文档](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) 声称支持 ROS 2 Humble + Gazebo Harmonic
   - 但 GitHub 的 `humble` 分支实际仍依赖 Gazebo Classic

## 解决方案

### 方案概述

使用 TurtleBot3 的 **feature-gazebo-sim-migration** 分支（开发中的 Gazebo Harmonic 迁移分支），并手动修改依赖以适配系统环境。

---

### 步骤 1: 安装 Navigation2

Navigation2 不依赖 Gazebo，可以直接从 apt 安装：

```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

**验证安装**：
```bash
ros2 pkg list | grep nav2
```

---

### 步骤 2: 创建统一的 TurtleBot3 工作目录

为了保持项目整洁，将所有 TurtleBot3 相关包放在一个统一的文件夹中：

```bash
cd ~/ros2_ws/src
mkdir turtlebot3_ws
cd turtlebot3_ws
```

---

### 步骤 3: 克隆 TurtleBot3 源码

克隆 TurtleBot3 的三个核心仓库（`humble` 分支）：

```bash
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

**目录结构**：
```
~/ros2_ws/src/
└── turtlebot3_ws/
    ├── turtlebot3/
    ├── turtlebot3_simulations/
    └── turtlebot3_msgs/
```

---

### 步骤 4: 切换到 Gazebo Harmonic 迁移分支

```bash
cd ~/ros2_ws/src/turtlebot3_ws/turtlebot3_simulations
git checkout feature-gazebo-sim-migration
```

**查看分支**：
```bash
git branch
# 输出: * feature-gazebo-sim-migration
```

---

### 步骤 5: 修改 package.xml 依赖

**关键步骤**：修改 `turtlebot3_gazebo` 包的依赖，使其匹配系统中实际安装的 ROS-Gazebo 桥接包。

#### 5.1 检查系统已安装的桥接包

```bash
dpkg -l | grep ros-humble-ros-gz
```

应该看到：
```
ros-humble-ros-gzharmonic-bridge
ros-humble-ros-gzharmonic-image
ros-humble-ros-gzharmonic-sim
```

#### 5.2 编辑 package.xml

文件路径：`~/ros2_ws/src/turtlebot3_ws/turtlebot3_simulations/turtlebot3_gazebo/package.xml`

**修改前**（第 22-24 行）：
```xml
<depend>ros_gz_bridge</depend>
<depend>ros_gz_image</depend>
<depend>ros_gz_sim</depend>
```

**修改后**：
```xml
<depend>ros_gzharmonic_bridge</depend>
<depend>ros_gzharmonic_image</depend>
<depend>ros_gzharmonic_sim</depend>
```

**为什么要修改**：
- GitHub 源码使用通用名称 `ros_gz_*`
- 你的系统使用特定版本 `ros_gzharmonic_*`（专门适配 Gazebo Harmonic）
- 包名必须匹配，否则编译时找不到依赖

---

### 步骤 6: 安装 Python 依赖

ROS 2 编译需要特定版本的 Python 模块：

```bash
# 安装 empy（ROS 消息生成器）
pip3 install "empy<4.0"

# 安装 lark（ROS IDL 解析器）
pip3 install lark
```

**注意**：
- `empy 4.x` 不兼容 ROS 2 Humble，必须使用 `3.x` 版本
- 如果之前安装了错误版本：`pip3 uninstall empy && pip3 install "empy<4.0"`

---

### 步骤 7: 编译 TurtleBot3 包

#### 7.1 清理旧的编译文件（可选，但推荐）

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
```

#### 7.2 编译 TurtleBot3 核心包

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
    turtlebot3_msgs \
    turtlebot3_description \
    turtlebot3_teleop \
    turtlebot3_node \
    turtlebot3_bringup \
    turtlebot3_cartographer \
    turtlebot3_navigation2 \
    turtlebot3_gazebo
```

**编译选项说明**：
- `--symlink-install`：创建符号链接而不是复制文件（修改源码后无需重新编译）
- `--packages-select`：只编译指定的包

#### 7.3 跳过的包

以下包依赖 Gazebo Classic，无需安装：
- `turtlebot3_fake_node`
- `turtlebot3_manipulation_gazebo`（如果报错可跳过）

---

### 步骤 8: 配置环境变量

编辑 `~/.bashrc` 添加：

```bash
# TurtleBot3 模型设置
export TURTLEBOT3_MODEL=burger

# Source ROS 2 工作空间
source ~/ros2_ws/install/setup.bash
```

应用更改：
```bash
source ~/.bashrc
```

---

## 验证安装

### 1. 检查 TurtleBot3 包

```bash
ros2 pkg list | grep turtlebot3
```

应该看到：
```
turtlebot3_bringup
turtlebot3_cartographer
turtlebot3_description
turtlebot3_gazebo
turtlebot3_msgs
turtlebot3_navigation2
turtlebot3_node
turtlebot3_teleop
```

### 2. 测试启动 Gazebo 仿真

#### 2.1 启动空白世界

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**预期结果**：
- Gazebo Harmonic 窗口打开
- TurtleBot3 Burger 机器人出现在场景中
- 终端显示桥接成功信息：
  ```
  [ros_gz_bridge]: Creating GZ->ROS Bridge: [clock ...]
  [ros_gz_bridge]: Creating GZ->ROS Bridge: [odom ...]
  [ros_gz_bridge]: Creating ROS->GZ Bridge: [cmd_vel ...]
  ```

#### 2.2 检查 ROS 话题

在新终端中：
```bash
ros2 topic list
```

应该看到：
```
/clock
/cmd_vel
/imu
/joint_states
/odom
/scan
/tf
```

### 3. 测试键盘控制

在新终端中启动遥控节点：
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

使用 `W/A/S/D/X` 键控制机器人移动。

### 4. 测试 Navigation2

#### 4.1 启动 TurtleBot3 World

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### 4.2 启动 Navigation2

在新终端中：
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

#### 4.3 使用 RViz2 设置导航目标

- 在 RViz2 中点击 "2D Pose Estimate" 设置初始位姿
- 点击 "Nav2 Goal" 设置目标点
- 机器人应该自动规划路径并导航

---

## 常见问题

### Q1: 编译时提示找不到 `ros_gz_bridge`

**原因**：未修改 `package.xml` 文件中的依赖。

**解决**：按照步骤 5 修改依赖为 `ros_gzharmonic_bridge`。

---

### Q2: 启动仿真时 Gazebo 崩溃

**可能原因**：
1. Gazebo Classic 和 Gazebo Harmonic 冲突
2. 显卡驱动问题

**检查 Gazebo 版本**：
```bash
gz sim --version
# 应该输出: Gazebo Sim, version 8.x.x
```

**如果同时安装了两个版本的 Gazebo**：
```bash
# 移除旧版 Gazebo
sudo apt purge -y gazebo11 libgazebo11*
sudo apt autoremove -y
```

---

### Q3: `empy` 模块版本错误

**错误信息**：
```
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
```

**解决**：
```bash
pip3 uninstall empy
pip3 install "empy<4.0"
```

---

### Q4: 源码必须一直保留吗？

**是的！** 因为使用了 `--symlink-install` 参数：
- `install/` 文件夹中的文件是符号链接，指向 `src/` 中的源文件
- 删除 `src/turtlebot3_ws/` 会导致链接失效

**验证符号链接**：
```bash
ls -la ~/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch/
# 应该看到 -> 指向 src/ 的链接
```

---

### Q5: 如何卸载 TurtleBot3？

```bash
# 删除编译文件
cd ~/ros2_ws
rm -rf build/turtlebot3* install/turtlebot3* log/

# 删除源码（可选）
rm -rf src/turtlebot3_ws/

# 重新编译工作空间
colcon build --symlink-install
```

---

## 工作原理说明

### 为什么需要源码编译？

1. **apt 包依赖旧版 Gazebo**
   - `ros-humble-turtlebot3-gazebo` → 依赖 `gazebo_ros_pkgs` → 依赖 Gazebo Classic 11

2. **GitHub 有 Gazebo Harmonic 支持分支**
   - `feature-gazebo-sim-migration` 分支专门为新版 Gazebo 重写
   - 使用 `ros_gz_*` 桥接包（新架构）

3. **包名适配**
   - 通用名称：`ros_gz_bridge`、`ros_gz_image`、`ros_gz_sim`
   - Harmonic 特定版本：`ros_gzharmonic_bridge`、`ros_gzharmonic_image`、`ros_gzharmonic_sim`
   - 需要手动修改 `package.xml` 以匹配系统实际安装的包

### ROS-Gazebo 桥接机制

```
ROS 2 话题                ros_gzharmonic_bridge              Gazebo 话题
-----------               ---------------------              -----------
/cmd_vel      ─────────>  parameter_bridge     ─────────>   /cmd_vel
/odom         <─────────  (YAML 配置)          <─────────   /model/.../odom
/scan         <─────────                        <─────────   /model/.../scan
```

桥接包的作用：
- 将 ROS 2 消息类型转换为 Gazebo 消息类型
- 实现双向通信（ROS ↔ Gazebo）

---

## 项目结构

```
~/ros2_ws/
├── src/
│   ├── turtlebot3_ws/                    # TurtleBot3 统一目录
│   │   ├── turtlebot3/                   # 核心包（描述、节点等）
│   │   ├── turtlebot3_msgs/              # 消息定义
│   │   └── turtlebot3_simulations/       # 仿真包
│   │       ├── turtlebot3_gazebo/        # Gazebo 世界和启动文件
│   │       │   ├── launch/               # 启动文件
│   │       │   ├── models/               # 机器人模型
│   │       │   ├── worlds/               # Gazebo 世界文件
│   │       │   └── package.xml           # ⚠️ 已修改依赖
│   │       └── turtlebot3_fake_node/     # 假节点（未使用）
│   └── my_robot_bringup/                 # 你的其他项目
├── build/                                # 编译中间文件
├── install/                              # 编译输出（符号链接到 src/）
└── log/                                  # 编译日志
```

---

## 相关资源

### 官方文档
- [TurtleBot3 仿真文档](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [Navigation2 文档](https://navigation.ros.org/)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)
- [ros_gz 桥接文档](https://github.com/gazebosim/ros_gz)

### GitHub 仓库
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [TurtleBot3 Messages](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)

### 关键分支
- `humble`：稳定分支，但依赖 Gazebo Classic
- `feature-gazebo-sim-migration`：开发分支，支持 Gazebo Harmonic（本指南使用）

---

## 重要发现总结

1. ✅ **TurtleBot3 确实支持 Gazebo Harmonic**，但需要使用开发分支
2. ✅ **apt 仓库中的包尚未更新**，必须从源码编译
3. ✅ **包名不匹配问题**需要手动修改 `package.xml`
4. ✅ **符号链接编译**要求源码必须保留
5. ✅ **Python 依赖版本**必须严格匹配（`empy < 4.0`）

---

## 常用命令速查

```bash
# 启动空白世界
ros2 launch turtlebot3_gazebo empty_world.launch.py

# 启动 TurtleBot3 世界
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 启动 TurtleBot3 House
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# 键盘控制
ros2 run turtlebot3_teleop teleop_keyboard

# 启动 Navigation2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

# 启动 Cartographer SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# 查看话题
ros2 topic list
gz topic -l

# 查看节点图
rqt_graph

# 检查编译的包
ros2 pkg list | grep turtlebot3
```

---

**创建日期**: 2025-10-21
**ROS 版本**: ROS 2 Humble
**Gazebo 版本**: Gazebo Harmonic LTS (8.9.0)
**系统**: Ubuntu 22.04
**关键分支**: feature-gazebo-sim-migration

---

## 补充说明：Cartographer 依赖

### 问题
在使用 Cartographer SLAM 功能时，可能遇到以下错误：

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

报错：
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
"package 'cartographer_ros' not found"
```

### 原因
`turtlebot3_cartographer` 包依赖 `cartographer_ros`，但该包未包含在基础安装中。

### 解决方案

安装 Cartographer 相关包：

```bash
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros
```

安装的主要包：
- `ros-humble-cartographer`：核心 SLAM 库
- `ros-humble-cartographer-ros`：ROS 2 桥接包
- `ros-humble-cartographer-ros-msgs`：消息定义
- `ros-humble-pcl-conversions`：点云转换工具

### 验证安装

```bash
# 检查包是否安装
dpkg -l | grep cartographer

# 启动 Cartographer SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

成功后应该看到：
- RViz2 窗口打开
- 实时构建 2D 地图
- 机器人位姿跟踪正常

**注意**：安装 Cartographer 会同时安装大量依赖（约 180 MB），包括 PCL（点云库）、VTK（可视化工具包）、OpenJDK 11 等。
