# Claude Code 使用说明

## 重要提醒

### hzh的密码：huangzihao1

### 1. 工作目录
- **始终在 `/home` 目录下工作**，不要局限于当前项目文件夹 `/home/hzh/ros2_ws/src`
- 用户的主要文件都在 `/home/hzh/` 目录下
- 执行命令时要考虑完整的文件路径

### 2. 避免循环执行命令
- **绝对不要执行会一直循环运行的命令**
- 对于需要持续运行的命令，添加超时限制：
  - 使用 `timeout Xs command` (例如：`timeout 5s ros2 topic pub`)
  - 使用 `--once` 参数 (例如：`ros2 topic pub --once`)
  - 使用有限次数的循环而不是无限循环

### 3. ROS 相关配置
- 系统安装了 ROS2 Humble：`/opt/ros/humble/`
- 需要先 `source /opt/ros/humble/setup.bash`
- 已安装的 MCP 服务器位置：`/home/hzh/ros-mcp-server/`

### 4. 启动节点问题
- 不要用 bash 工具启动需要持续运行的节点（会卡住）
- 该mcp服务需要用户手动启动节点，然后其它的命令语句交给claude code
- 建议用户手动启动这些节点：
  - turtlesim: `ros2 run turtlesim turtlesim_node`
  - rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

### 5. 其他重要信息
- 用户使用 Claude Code 的 MCP 功能，不是 Claude Desktop
- 在执行任何持续运行的操作前，要考虑如何优雅地停止或限制执行时间

## 常用命令模板

### 安全的 ROS 话题发布
```bash
# 单次发布
ros2 topic pub --once /topic_name msg_type "data"

# 限时发布 (5秒后自动停止)
timeout 5s ros2 topic pub --rate 10 /topic_name msg_type "data"
```

### 检查系统状态
```bash
# 检查节点
source /opt/ros/humble/setup.bash && ros2 node list

# 检查话题
source /opt/ros/humble/setup.bash && ros2 topic list
```