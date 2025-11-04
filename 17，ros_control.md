
### 1，定义
![Pasted image 20251009194110.png](<attachments/Pasted image 20251009194110.png>)

整体架构：

  应用层 (你的代码)
      ↓ 发送速度命令 (/cmd_vel)，比如linear.x = 0.5 m/s, angular.z = 0.2 rad/s
  控制器管理器 (controller_manager)
      ↓ 调用
  差速控制器 (diff_drive_controller)，比如根据输入，计算得出，左轮速度 = 0.4 rad/s，右轮速度 = 0.6 rad/s
      ↓ 计算并发送关节命令
  硬件接口 (Hardware Interface)
      ↓ write() / read()
  硬件驱动 (例如串口驱动)
      ↓ 通信协议
  实际硬件 (左右轮电机)


![Pasted image 20251009195143.png](<attachments/Pasted image 20251009195143.png>)





当没有实际硬件的时候，可以使用这个mock component来模拟一个硬件组件
![Pasted image 20251009194429.png](<attachments/Pasted image 20251009194429.png>)

### 2，控制器和硬件接口之间的“接口规范合同”

是一个xacro，放在urdf里面
![Pasted image 20251009194715.png](<attachments/Pasted image 20251009194715.png>)
```xml
<?xml version="1.0"?>

<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  

<!--名字无所谓，类型其实有各种，但是system包括了他们全部，所以暂时先用syste就行-->

	<ros2_control name="MobileBaseHardwareInterface" type="system" >

		<hardware>

			<plugin>mock_component/GenericSystem</plugin> <!-- 硬件接口名字，这里先用mock_component代替 -->

			<param name="calculate_dynamics">true</param>

		</hardware>

		<joint name="base_right_wheel_joint"> <!-- 你想控制哪个关节 -->

			<command_interface name="velocity" /> <!-- 你要通过什么参数来控制这个关节 -->

			<state_interface name="position" />

			<state_interface name="velocity" /> <!-- 状态参数（闭环控制） -->

		</joint>

		<joint name="base_left_wheel_joint"> <!-- 你想控制哪个关节 -->

			<command_interface name="velocity" /> <!-- 你要通过什么参数来控制这个关节 -->

			<state_interface name="position" />

			<state_interface name="velocity" /> <!-- 状态参数（闭环控制） -->

		</joint>

	</ros2_control>

  

</robot>
```

它的作用：
1. 告诉 ros2_control 系统：
    - 我有哪些硬件（左轮、右轮）
    - 硬件接口插件是什么（mock_component）
    - 可以发送什么命令（velocity）
    - 可以读取什么状态（position, velocity）
  2. 作为桥梁：
    - 让控制器知道"可以发送速度命令"
    - 让硬件接口知道"需要接收速度命令并反馈位置和速度"



###  3，定义控制器

其实就是定义了manager和具体控制器（有哪些控制器，控制器具体参数的配置）
一个yaml文件
![Pasted image 20251010152953.png](<attachments/Pasted image 20251010152953.png>)

https://github.com/ros-controls/ros2_controllers/tree/humble
这个是所有的现成控制器

我们代码里面的type对应网站xml里面的name：
![Pasted image 20251010153544.png](<attachments/Pasted image 20251010153544.png>)
网站里面的yaml对应具体的参数配置：
![Pasted image 20251010153621.png](<attachments/Pasted image 20251010153621.png>)



```yaml
controller_manager:

 ros__parameters:

  update_rate: 50

  

  joint_state_broadcaster:

    type: joint_state_broadcaster/JointStateBroadcaster

  

  diff_drive_controller:

    type: diff_drive_controller/DiffDriveController

diff_drive_controller:

  ros__parameters:

   left_wheel_names: ["base_left_wheel_joint"]

   right_wheel_names: ["base_right_wheel_joint"]

  

   wheel_separation: 0.45

   wheel_radius: 0.1

  

   odom_frame_id: "odeom"

   base_frame_id: "base_footlink"

  

   pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

   twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

  

   enable_odom_tf: true

   publish_rate: 50.0

  

   linear.x.has_velocity_limits: true

   linear.x.max_velocity: 1.0

   linear.x.min_velocity: -1.0

  

   angular.z.has_velocity_limits: true

   angular.z.max_velocity: 1.0

   angular.z.min_velocity: -1.0
```



### 4，手动启动流程

其实这个手动流程跟[[13，完成输出tf的流程]]差不多

（1），启动robot_state_publisher
(跟以前一样)
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/hzh/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"
```

![Pasted image 20251010153907.png](<attachments/Pasted image 20251010153907.png>)

（2），启动controller_manager
（之前说过，manager可以把控制器跟硬件接口整合到一起）
注意，这里不仅要--ros-args，我们的控制器文件yaml，还要参数我们的整体urdf！


```
ros2 run controller_manager ros2_control_node --ros-args --params-file /home/hzh/ros2_ws/src/my_robot_bringup/config/my_robot_controllers.yaml -p robot_description:="$(xacro /home/hzh/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"

```


![Pasted image 20251010154411.png](<attachments/Pasted image 20251010154411.png>)

（3），启动具体的控制器
根据yaml文件里面写的控制器名字，来手动启动具体的控制器

```
ros2 run controller_manager spawner joint_state_broadcaster

ros2 run controller_manager spawner diff_drive_controller

```

![Pasted image 20251010154652.png](<attachments/Pasted image 20251010154652.png>)


（4），启动rviz
（跟以前一样）

```
ros2 run rviz2 rviz2 -d /home/hzh/ros2_ws/src/my_robot_bringup/rviz/urdf_config.rviz
```

![Pasted image 20251010154803.png](<attachments/Pasted image 20251010154803.png>)

（5），到这里其实就已经全部ok了，接下来测试一下

*其实从这里就能看出来，我们这个ros_control的控制器，效果跟之前gazebo的插件差不多，只不过gazebo的插件只能在gazebo里面生效（导出到ros还得bridge一下），而这个控制器的通用性更好。*

![Pasted image 20251010164739.png](<attachments/Pasted image 20251010164739.png>)

![Pasted image 20251010155217.png](<attachments/Pasted image 20251010155217.png>)

![Pasted image 20251010155256.png](<attachments/Pasted image 20251010155256.png>)

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```
注意这里cmd_vel的名字要重新映射一下，而且要加上后面那个参数，代表加上时间戳

然后就能通过键盘控制机器人了
![Pasted image 20251010155415.png](<attachments/Pasted image 20251010155415.png>)


### 5，用启动文件启动整个流程

其实就是把上面手动的过程，写成launch文件，跟以前是一样的

![Pasted image 20251010165626.png](<attachments/Pasted image 20251010165626.png>)

```xml
<launch>

	<let name="urdf_path" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro" />

	<node pkg="robot_state_publisher" exec="robot_state_publisher">

		<param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />

	</node>

  

<!-- 启动manager，输入控制器定义yaml 和 urdf -->

	<node pkg="controller_manager" exec="ros2_control_node">

		<param from="$(find-pkg-share my_robot_bringup)/config/my_robot_controllers.yaml" />

		<param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />

	</node>

  

	<!-- 创建生成两个控制器 -->

	<node pkg="controller_manager" exec="spawner" args="joint_state_broadcaster" />

  

	<node pkg="controller_manager" exec="spawner" args="diff_drive_controller" />

  

	<let name="rviz_config" value="$(find-pkg-share my_robot_bringup)/rviz/urdf_config.rviz" />

	<node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config)" />

  

</launch>

```


因为这里用到了controller_manager这个包，所以需要在package.xml里面写一个新的依赖！
![Pasted image 20251010165825.png](<attachments/Pasted image 20251010165825.png>)


### 6，常用的control命令

（1），查看现在的控制器列表
```
ros2 control list_controllers
```
![Pasted image 20251010170217.png](<attachments/Pasted image 20251010170217.png>)

（2），列出所有可能可用的控制器（在网站上也能找到）
```
ros2 control list_controller_types 
```
![Pasted image 20251010170303.png](<attachments/Pasted image 20251010170303.png>)

（3），查看现在的硬件接口

其实就是我们在mobile_base.ros2_control.xacro中定义的硬件接口
```
ros2 control list_hardware_interfaces
```

![Pasted image 20251010170600.png](<attachments/Pasted image 20251010170600.png>)
可以看到command interfaces（正向传输）和state interfaces（闭环反向反馈），对于command interfaces来说，这个claimed就说明它已经被某一个控制器占用了（我们的diffdrive定义的时候定义了这两个硬件接口），它只能被一个控制器占用。
而对于state interfaces来说，谁都可以调用他的信息


（4），查看硬件接口的”合同“

其实就是我们在mobile_base.ros2_control.xacro中定义的合同名字...那些信息

```
ros2 control list_hardware_components
```

![Pasted image 20251010171107.png](<attachments/Pasted image 20251010171107.png>)


### 7，硬件驱动

*其实硬件驱动就是：根据硬件说明书，通过通信协议（串口/can/485等）读写硬件的寄存器/发送命令，实现控制功能*


通过封装成函数，只提供给外界简单的数据类型接口

ROS2 Hardware Interface (ros2_control)
           ↓ 调用简单函数
      硬件驱动 API/库 (封装层)
           ├─ set_velocity(left_rpm, right_rpm)
           ├─ get_velocity()
           ├─ enable_motor()
           └─ disable_motor()
           ↓ 内部操作寄存器
      Modbus/RS485 通信
           ↓
      ZLAC8015D 驱动器
           ↓
        轮毂电机


### 8，创建硬件接口包

（1），在代码区中创建一个硬件接口的包
![Pasted image 20251011145728.png](<attachments/Pasted image 20251011145728.png>)
```
ros2 pkg create my_robot_hardware --build-type ament_cmake --dependencies rclcpp
```

创建好之后应该是这样的
![Pasted image 20251011145918.png](<attachments/Pasted image 20251011145918.png>)


（2），把硬件驱动
文件放到include文件夹底下
![Pasted image 20251011145944.png](<attachments/Pasted image 20251011145944.png>)


（3），修改cmake和package.xml

cmake中把多余的test部分删掉（跟以前一样），然后因为我们这个硬件驱动用到了一个库，叫做dynamixel_sdk，所以需要
```
find_package(dynamixel_sdk_REQUIRED)
```

并且因为我们硬件驱动放在include文件夹底下，所以需要
```
include_directories(include)
```

![Pasted image 20251011150256.png](<attachments/Pasted image 20251011150256.png>)




package.xml需要加上我们依赖的库
```
<depend>dynamixel_sdk</depend>
```

![Pasted image 20251011150309.png](<attachments/Pasted image 20251011150309.png>)

还有一点！因为我们用到的是c++，所以依赖路径处理比较麻烦，*需要在.vscode里面，有个includepath中添加我们include文件夹的路径！！*
![Pasted image 20251011192721.png](<attachments/Pasted image 20251011192721.png>)

### 9，编写硬件接口文件,hpp和cpp

- 1,首先需要理解一个概念，就是*硬件接口这部分其实是一个生命节点*，也就是他会自动进行图片中的步骤：
	激活的第一步，一般是简单的初始化，也就是跟硬件连接上
	激活的第二部，一般是激活电机
	反向关闭的话就是反过来
	
	![Pasted image 20251011153628.png](<attachments/Pasted image 20251011153628.png>)

- 2，*其次就是我们的硬件接口文件，是通过  头文件.hpp    和  源文件.cpp  构成的*
	Hardware Interface (硬件接口) = .hpp + .cpp
	  ├─ mobile_base_hardware_interface.hpp (头文件)
	  │   ├─ 类声明
	  │   ├─ 函数声明（接口）
	  │   └─ 成员变量声明
	  │
	   |─ mobile_base_hardware_interface.cpp (源文件)
      └─ 函数实现（具体代码）
	
	也就是.hpp负责，告诉别人有什么，有哪些函数，有哪些变量
	然后.cpp负责，具体实现，每一部分函数内部有什么
	
	然后调用关系其实是cpp->hpp->drive。也就是硬件接口调用硬件驱动

- 3，写hpp头文件
	在include文件夹底下写hpp
	（一些依赖，在上面第八步已经完成了）
	mobile_base_hardware_interface.hpp
	![Pasted image 20251011192905.png](<attachments/Pasted image 20251011192905.png>)

- 4，写cpp源文件
	在src文件夹底下写cpp
	mobile_base_hardware_interface.cpp
	![Pasted image 20251011193036.png](<attachments/Pasted image 20251011193036.png>)
	![Pasted image 20251011193058.png](<attachments/Pasted image 20251011193058.png>)
	![Pasted image 20251011193106.png](<attachments/Pasted image 20251011193106.png>)



### 10，写硬件接口 插件描述文件(类似于注册表)

在硬件接口这个整个包底下，创建一个xml文件
my_robot_hardware_interface.xml
（跟cmake平级）

![Pasted image 20251012162827.png](<attachments/Pasted image 20251012162827.png>)


### 11，在package和cmake中再次添加依赖的库

- 1，在package中添加的
```xml
<depend>rclcpp</depend>

<depend>rclcpp_lifecycle</depend>

<depend>hardware_interface</depend>

<depend>pluginlib</depend>

<depend>dynamixel_sdk</depend>
```
![Pasted image 20251012163328.png](<attachments/Pasted image 20251012163328.png>)

- 2，在cmake里面添加的是
```txt
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)

find_package(hardware_interface REQUIRED)

find_package(pluginlib REQUIRED)

find_package(dynamixel_sdk_REQUIRED)
```

![Pasted image 20251012163415.png](<attachments/Pasted image 20251012163415.png>)

像dynamixel_sdk是我们之前添加硬件驱动时候，硬件驱动所依赖的库
然后rclcpp_lifecycle是因为我们的硬件接口是生命节点
hardware_interface是因为hpp依赖这个库
pluginlib是我们在cpp的最后加入的插件库


- 3，*cmake中还需要添加一大串内容，不过大部分都是固定的*

我没有加括号的地方，就不需要改动
```
add_library(

${PROJECT_NAME}

SHARED

src/mobile_base_hardware_interface.cpp(这里就是你这个cpp的名字)

)

  

ament_target_dependencies(${PROJECT_NAME} rclcpp clcpp_lifecycle hardware_interface pluginlib dynamixel_sdk)    （你所依赖的库）

  

pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)  （这个xml文件，就是cmake到xml的路径，因为是平级）

  

install(

TARGETS ${PROJECT_NAME}

DESTINATION lib

)

  

install(

DIRECTORY include

DESTINATION include

)

  

ament_export_libraries(

${PROJECT_NAME}

)

  

ament_export_dependencies(

hardware_interface

pluginlib

rclcpp

rclcpp_lifecycle

)   （你的依赖库）
```

![Pasted image 20251012165307.png](<attachments/Pasted image 20251012165307.png>)


### 12，把我们原来控制器用的mock虚拟硬件接口，替换成我们自己写的硬件接口

![Pasted image 20251012170425.png](<attachments/Pasted image 20251012170425.png>)

这个名字就是我们注册表文件里面的name
![Pasted image 20251012170449.png](<attachments/Pasted image 20251012170449.png>)

### 13，运行

运行成功之后应该是这样的![Pasted image 20251012171744.png](<attachments/Pasted image 20251012171744.png>)

如果没有插入真实硬件就是
![Pasted image 20251012171834.png](<attachments/Pasted image 20251012171834.png>)
查看info可以看到on_init成功了，因为这个初始化只是urdf和变量的初始化
然后发现on_configure失败了，因为我们没有真实的硬件，所以硬件的初始化连接失败
后面的步骤就无法进行



### 14，在硬件接口中添加参数

在cpp中通过,引用参数
```
info_.hardware_parameters["left_motor_id"]
```
注意这里引用来的是一个字符串，所以如你需要数字的话，需要转换一下格式
```
std::stoi(info_.hardware_parameters["left_motor_id"])
```
string to int
![Pasted image 20251013161325.png](<attachments/Pasted image 20251013161325.png>)





在control的urdf中，通过在加载硬件插件底下加入参数，来设置参数
```
<param name="left_motor_id">10</param>

<param name="right_motor_id">20</param>

<param name="dynamixel_port">/dev/ttyACM0</param>
```
![Pasted image 20251013161303.png](<attachments/Pasted image 20251013161303.png>)

### 15，总结

飞书文档
https://ccnj3kuvti9y.feishu.cn/docx/KjU2dBdoDoklvcxixVdciMbkn9e

![Pasted image 20251013161632.png](<attachments/Pasted image 20251013161632.png>)
![Pasted image 20251013161644.png](<attachments/Pasted image 20251013161644.png>)