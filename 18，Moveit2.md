
### 1,定义
![Pasted image 20251019150038.png](<attachments/Pasted image 20251019150038.png>)
moveit会帮忙完成：
	基础的逆运动学
	运动规划，路径规划
	碰撞检测


### 2，安装moveit

https://moveit.ai/install-moveit2/binary/
官方网站上面有安装语句
```
sudo apt install ros-humble-moveit
```

除了安装moveit以外，还要安装一个
```
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

然后在bashrc里面加入一个环境变量
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


### 3，启动moveit配置助手

- 1,启动配置的助手
```
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

- 2,选择你的完整urdf文件，创建一个新的moveit包
	![Pasted image 20251013193229.png](<attachments/Pasted image 20251013193229.png>)
	
	（注意！！可能会遇到加载urdf模型的时候闪退的情况，我的解决方法是，在urdf文件里面写好惯量部分，并且重要的是，重新构建一下环境，再试试就可以了！！！）

- 3，self-collisions碰撞检测部分（第二个按钮）
	这里其实是碰撞检测
	
	重点是点一下这个按钮
	![Pasted image 20251013194148.png](<attachments/Pasted image 20251013194148.png>)
	他会生成一个碰撞矩阵
	
	后面那个Adjacent说明这两个 link 是相邻的，检测它们之间的碰撞没有意义，因为它们本来就应该靠在一起
	
	Never in Collision说明这些link 在所有可能的运动范围内都不会发生碰撞，所以可以忽略
	
	其实这一步的作用就是让他计算并记录了一下哪些碰撞可以忽略，这样能提高他之后计算的效率


- 4，virtual joints虚拟关节部分（第三个按钮）
	其实就是添加一个我模型根link和世界坐标系直接的虚拟关节联系，方便之后进行机器人跟坐标系和世界坐标系的转换
	![Pasted image 20251013195144.png](<attachments/Pasted image 20251013195144.png>)
	![Pasted image 20251013194953.png](<attachments/Pasted image 20251013194953.png>)
	Virtual Joint（虚拟关节）的作用：
	1. 连接机器人和世界：
    - 把你的机器人根 link（base_link）连接到世界坐标系（world）
    - 这样 MoveIt 就知道机器人在世界中的位置关系
    2. 定义机器人的移动方式：
    - fixed：机器人固定在地面上（如工业机械臂）
    - floating：机器人可以在 3D 空间自由移动和旋转（如无人机、漂浮机器人）
    - planar：机器人在平面上移动（如移动底盘，x、y、旋转）
    3. 坐标变换：
    - 提供了从世界坐标系到机器人坐标系的变换关系
    - MoveIt 需要这个来进行路径规划和碰撞检测

- 5，planning groups（第四个按钮）
	Planning Groups（规划组）就是你想让 MoveIt 控制和规划的关节集合。
	其实就是选中哪些关节/部件，你要依靠它的运动学库来自动求解的
	![Pasted image 20251013195247.png](<attachments/Pasted image 20251013195247.png>)
	![Pasted image 20251013195524.png](<attachments/Pasted image 20251013195524.png>)
	![Pasted image 20251013195542.png](<attachments/Pasted image 20251013195542.png>)
	![Pasted image 20251013195624.png](<attachments/Pasted image 20251013195624.png>)
	![Pasted image 20251013195804.png](<attachments/Pasted image 20251013195804.png>)



- 6，robot pose（第五个按钮）
	其实就是定义一些机器人的预设点位
	![Pasted image 20251013200143.png](<attachments/Pasted image 20251013200143.png>)
	![Pasted image 20251013200205.png](<attachments/Pasted image 20251013200205.png>)


- 7，end effectors（第六个按钮）
	添加末端执行器
	我们这里暂时没有
	
	在添加末端夹爪的时候，这里需要配置
	注意的就是这个parent link，是整个夹爪的父link，所以是tool_link
	后面那个parent group可选可不选
	![Pasted image 20251016201450.png](<attachments/Pasted image 20251016201450.png>)


- 8，passive joints（第七个按钮）
	定义被动关节
	我们这里暂时没有


- 9，ros2_control urdf modifications（第八个按钮）
	就是他能生成一个关于ros_control中关于command interface和state interface的文件
	详见[[17，ros_control]]的第二步
	就是command interface是你要发送给电机的指令，你要怎么控制它
	而state interface是电机发回来的数据
	![Pasted image 20251013201036.png](<attachments/Pasted image 20251013201036.png>)

- 10，ros2 controllers（第九个按钮）
	点一下这个按钮，他就能生成ros_control所用的插件文件
	![Pasted image 20251013201306.png](<attachments/Pasted image 20251013201306.png>)


- 11，moveit controllers（第十个按钮）
	![Pasted image 20251013201450.png](<attachments/Pasted image 20251013201450.png>)


- 12，perception传感器配置（第十一个按钮）
	如果你有相机或者雷达之类的传感器，可以在这里选择加入点云图或者深度图
	我们这里暂时没有


- 13，launch files（第十二个按钮）
	启动文件配置
	我们这里不需要进行改动


- 14，author information（第十三个按钮）
	写入作者信息，最好还是写一下，要不然可能会出错
	![Pasted image 20251013201845.png](<attachments/Pasted image 20251013201845.png>)


- 15，configuration files（第十四个按钮）
	选择一下保存的路径
	保存在src底下，然后新建一个文件夹叫做/my_robot_moveit_config
	![Pasted image 20251013202055.png](<attachments/Pasted image 20251013202055.png>)


- 16，最后生成完毕之后，检查一下文件生成了没有，再关闭助手
	![Pasted image 20251013202443.png](<attachments/Pasted image 20251013202443.png>)
	![Pasted image 20251013202455.png](<attachments/Pasted image 20251013202455.png>)


### 4，修改moveit的包

选我们之前moveit帮我们生成的包的路径就可以了
![Pasted image 20251013202853.png](<attachments/Pasted image 20251013202853.png>)


### 5，查看一下moveit的包里面有哪些东西（重要2,4修改）

- config文件夹里面
	1,initial_positions.yaml初始关节位姿文件
	![Pasted image 20251013205336.png](<attachments/Pasted image 20251013205336.png>)
	2,joint_limits.yaml速度和角速度极限的设置
	*注意！！这里的max最大值必须改成浮点数！！而且把joint6的速度控制改成true，最大速度也写成1.0！！而且对于每个关节的角速度控制也打开，最大角速度设为1.0！！*
	![Pasted image 20251013212218.png](<attachments/Pasted image 20251013212218.png>)
	3,kinematics.yaml运动学库
	默认不动
	4,moveit_controllers.yaml
	*注意！!要加入两行
	action_ns: follow_joint_trajectory
	default: true*
	![Pasted image 20251013212921.png](<attachments/Pasted image 20251013212921.png>)
	5,moveit.rviz
	默认不动
	6,my_robot.ros2_control.xacro控制器和硬件接口的合同文件
	![Pasted image 20251013205823.png](<attachments/Pasted image 20251013205823.png>)
	7,*my_robot.srdf机械臂点位/碰撞忽略*
	![Pasted image 20251013210028.png](<attachments/Pasted image 20251013210028.png>)
	8,my_robot.urdf.xacro其实是moveit最后的完整urdf文件
	他引入了我们自己的最终urdf  +  ros_control的urdf文件
	![Pasted image 20251013210243.png](<attachments/Pasted image 20251013210243.png>)
	9,ros2_controllers.yaml控制器manager文件
	![Pasted image 20251013210433.png](<attachments/Pasted image 20251013210433.png>)

- launch文件夹里面
	1,move_group.launch.py启动moveit的文件，所以如果我们以后要自己写启动moveit的启动文件，需要引入这个启动文件
	![Pasted image 20251013210710.png](<attachments/Pasted image 20251013210710.png>)
	2,demo.launch.py测试整个文件夹的文件
	![Pasted image 20251013210847.png](<attachments/Pasted image 20251013210847.png>)
	3,move_group.launch.py是一个启动moveit功能的文件


### 6，常见的moveit操作
运行demo的那个文件，来测试整个包
- 1，plan
	给定当前位姿和目标位姿，然后plan他会生成一个路径规划的轨迹动画
	![Pasted image 20251013213237.png](<attachments/Pasted image 20251013213237.png>)
	如果你不想看这个重复的规划动画，可以在这里勾掉这个选项
	![Pasted image 20251013213342.png](<attachments/Pasted image 20251013213342.png>)


- 2,execute
	根据plan的规划，执行运动
	![Pasted image 20251016191146.png](<attachments/Pasted image 20251016191146.png>)
	当然也可以点底下的plan and execute


- 3,调速度
	![Pasted image 20251016191227.png](<attachments/Pasted image 20251016191227.png>)


- 4,也可以拉动这个来手动调整机械臂末端位置
	![Pasted image 20251016191333.png](<attachments/Pasted image 20251016191333.png>)


 - 5,*启用笛卡尔移动（直线路径的运动）*
	 但是可能会找不到路径，因为直线运动相对于正常的弧线运动约束更多
	![Pasted image 20251016191553.png](<attachments/Pasted image 20251016191553.png>)


- 6,显示规划的每一步轨迹
	也是在planned path里面
	![Pasted image 20251016191921.png](<attachments/Pasted image 20251016191921.png>)
	在robotmodel里面也可以设置类似的东西！
	![Pasted image 20251019161127.png](<attachments/Pasted image 20251019161127.png>)

- 7,*展示某一个部件的轨迹(不保留)*
	也是在planned path里面，然后links，找到想要的部件点开，然后show trail
	![Pasted image 20251016192112.png](<attachments/Pasted image 20251016192112.png>)
	![Pasted image 20251016192222.png](<attachments/Pasted image 20251016192222.png>)

- 8,*展示某一个部件的轨迹(保留)*
	这次是在scene robot里面，links，然后show trail
	![Pasted image 20251016192739.png](<attachments/Pasted image 20251016192739.png>)


### 7，配置自己的moveit启动包

- 1，把ros2_controllers.yaml的控制器文件放到bringup的config底下
	![Pasted image 20251016212823.png](<attachments/Pasted image 20251016212823.png>)

- 2，把my_robot.ros2_control.xacro这个硬件接口合同文件，放到urdf里面
	![Pasted image 20251016212948.png](<attachments/Pasted image 20251016212948.png>)


- 3，修改xacro硬件合同文件
	
	把开头的两行的macro去掉，倒数第二行也有一个macro也要删掉，也就是只留《ros_control》开始，并且改一个name
	
	把state interface的param的默认参数，都改成0.0
	![Pasted image 20251016213616.png](<attachments/Pasted image 20251016213616.png>)


- 4，编写启动文件
	其实跟以前的ros_control启动文件差不多
	![Pasted image 20251017142235.png](<attachments/Pasted image 20251017142235.png>)
	先启动robot_state_publisher
	再启动controller_manager
	再按照yaml文件里面写的各控制器的名字来启动各个控制器节点
	最后的rviz部分有点不一样
	*除了包含正常的rviz之外，还包含了/my_robot_moveit_config/launch/下的move_group.launch.py
	（这个move_group是启动moveit功能）*
	
	注意：之前的rviz配置，打开是没有moveit的，需要手动添加一下，然后保存rviz文件
	![Pasted image 20251017142518.png](<attachments/Pasted image 20251017142518.png>)


### 8，使用moveit的c++ api
其实就是通过c++代码控制moveit
![Pasted image 20251019150038.png](<attachments/Pasted image 20251019150038.png>)

- 1,创建c++的包
	![Pasted image 20251019150127.png](<attachments/Pasted image 20251019150127.png>)

- 2,写控制的代码
	![Pasted image 20251019150248.png](<attachments/Pasted image 20251019150248.png>)

- 3，记得修改cmake和package
	![Pasted image 20251019150405.png](<attachments/Pasted image 20251019150405.png>)
	![Pasted image 20251019150423.png](<attachments/Pasted image 20251019150423.png>)

- 4,*修改rviz配置，去除掉橙色部分*
	这一步是可选的，因为我们使用moveit的api之后，灰色代表model现在的状态，他只会动灰色的部分。而橙色的部分是我们刚刚的motion planning部分，他属于手动部分，不会动，所以为了美观，我们可以把那个部分remove掉，然后保存一下rviz配置
	![Pasted image 20251019150508.png](<attachments/Pasted image 20251019150508.png>)
	![Pasted image 20251019150759.png](<attachments/Pasted image 20251019150759.png>)
	![Pasted image 20251019150806.png](<attachments/Pasted image 20251019150806.png>)


- 5,【1】用设定好的点位名字，作为目标：
```c++
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
```


- 6,【2】用关节参数作为目标：
```c++
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
```


- 7,【3】用末端位姿（在基坐标系（也是手动选的））做目标
```c++
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
```


- 8,【4】笛卡尔直线路径：
```c++
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
```


### 9，写commander节点，使用moveit的c++ api
其实就是第八点是一个固定流程序，
*但是我们想要把这个moveit的api通过一个节点封装起来，然后这个节点又有订阅者。这样就能通过简单的话题，来传输简单的数据，来直接控制moveit*

程序/home/hzh/ros2_ws/src/my_robot_commander_cpp/src/commander_template.cpp

因为引入了新的依赖，并且是新的执行文件，所以注意修改cmake和package
![Pasted image 20251020224254.png](<attachments/Pasted image 20251020224254.png>)
![Pasted image 20251020224346.png](<attachments/Pasted image 20251020224346.png>)
注意我们这里用到了自己写的数据类型
![Pasted image 20251020224431.png](<attachments/Pasted image 20251020224431.png>)
所以在commander里面也要引入我们数据类型包依赖，所以ｃ＋＋的头文件也要写一下我们数据接口包的路径！
![Pasted image 20251020224609.png](<attachments/Pasted image 20251020224609.png>)
也在启动文件中加入这个节点
![Pasted image 20251020204835.png](<attachments/Pasted image 20251020204835.png>)


能发现我们订阅的话题
对这个话题发布内容就能直接控制moveit
![Pasted image 20251020204906.png](<attachments/Pasted image 20251020204906.png>)

![Pasted image 20251020224731.png](<attachments/Pasted image 20251020224731.png>)


### 10，总结

飞书文档：
https://ccnj3kuvti9y.feishu.cn/docx/TTYmdJD1eoBdqExPHONcm3Sgn9g

![Pasted image 20251021191028.png](<attachments/Pasted image 20251021191028.png>)