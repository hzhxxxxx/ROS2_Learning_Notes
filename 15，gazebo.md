gazebo是一种仿真平台，它是独立于ros的一个东西


- 1，安装新版gazebo garden
	详见：
	https://blog.csdn.net/weixin_49185284/article/details/147652706


- 2，启动gazebo
	（1），在终端直接启动
	![Pasted image 20251002165214.png](<attachments/Pasted image 20251002165214.png>)
	这样进去他会让你选环境配置，或者你知道环境配置之后可以这样进入
	![Pasted image 20251002165254.png](<attachments/Pasted image 20251002165254.png>)
	![Pasted image 20251002165305.png](<attachments/Pasted image 20251002165305.png>)
	
	（2），也可以通过ros的启动文件来启动gazebo
	ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
	![Pasted image 20251002165339.png](<attachments/Pasted image 20251002165339.png>)
	
	不过需要注意的是，因为gazebo是独立于ros的软件，所以暂时他俩的信息是不互通的：
	![Pasted image 20251002165422.png](<attachments/Pasted image 20251002165422.png>)


- 3，手动将ros与gazebo联系起来
	（1），将urdf导入robot_state_publisher并启动（这个地方详见[[13，完成输出tf的流程]]）
	![Pasted image 20251004150310.png](<attachments/Pasted image 20251004150310.png>)
	（2），通过ros launch启动gazebo
	![Pasted image 20251004150454.png](<attachments/Pasted image 20251004150454.png>)
	（这个-r 意思是启动时间流逝，其实就是gazebo左下角的启动/暂停健）
	
	（3），把ros里面的话题robot_description导入gazebo中
	![Pasted image 20251004150555.png](<attachments/Pasted image 20251004150555.png>)
	ros2 run ros_gz_sim create -topic robot_description
	
	首先这个关于这个robot_description是什么怎么来的，详见[[13，完成输出tf的流程]]
	大概就是他是robot_state_publisher所发出的机器人的模型蓝图文件
	然后我们通过这个语句，把蓝图导入到gazebo里面。
	注意这是一个一次性的动作，导入之后就不需要一直保持通讯了
	![Pasted image 20251004150833.png](<attachments/Pasted image 20251004150833.png>)
	
	（4），rviz可视化部分
	![Pasted image 20251004151125.png](<attachments/Pasted image 20251004151125.png>)
	详见[[13，完成输出tf的流程]]


- 4，通过launch文件，一次性完成启动gazebo的完整流程：
	
	其实就是对上面3手动过程的一次launch文件自动化流程整合
	
	写launch文件：![Pasted image 20251004154247.png](<attachments/Pasted image 20251004154247.png>)
	（注意这里这个启动gz_sim第二步，这个文件路径）
	![Pasted image 20251004154407.png](<attachments/Pasted image 20251004154407.png>)
	![Pasted image 20251004154423.png](<attachments/Pasted image 20251004154423.png>)
	
	注意这里用到很多个pkg，所以在package.xml里面要加入依赖！
	![Pasted image 20251004154516.png](<attachments/Pasted image 20251004154516.png>)

- 5，使用 YAML 配置文件启动 ros_gz_bridge
	
	[[12，urdf文件]]里面提到了在urdf文件里面写入gazebo插件，这会使得在gazebo侧多了一些特定的话题，但是因为ros和gazebo是独立的，所有我们需要一个叫ros_gz_bridge的节点，在两侧建立一个沟通的桥梁。
	![Pasted image 20251005135756.png](<attachments/Pasted image 20251005135756.png>)
	*通过把具体传输的话题写到yaml文件里面，然后当做参数放到ros_gz_bridge后面*
	
	![Pasted image 20251005135921.png](<attachments/Pasted image 20251005135921.png>)
	这个写法都是：
	-ros_topic_name: ""
	gz_topic_name: ""
	ros_type_name: ""
	gz_type_name: ""
	direction: 

	topic的名字通过：gz topic -l  和  ros2 topic list 查看
	![Pasted image 20251005140101.png](<attachments/Pasted image 20251005140101.png>)

	topic的类型：
	ros侧通过 ros2 topic info /clock 查看
	![Pasted image 20251005140201.png](<attachments/Pasted image 20251005140201.png>)
	gz侧通过  gz topic -i -t /world/empty/model/my_robot/joint_state  查看
	（-i 就是info的意思      -t 就是 topic的意思）
	![Pasted image 20251005140323.png](<attachments/Pasted image 20251005140323.png>)
	这里有两个数据类型的前缀，是因为我电脑里两个版本的gazebo，humble自动的gazebo是旧版本的，他的前缀是ignition。同时我使用的命令都是新版本的，前缀是gz，但是这个新版本有点不兼容ros-humble。
	这里我用的gz_type是 ignition.msgs.Model
	
	当然也可以先找gz侧数据类型，然后通过https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
	这个网站去找对应的ros侧数据类型
	![Pasted image 20251005140649.png](<attachments/Pasted image 20251005140649.png>)
	
	
	direction就是哪侧传到那侧的事情了。
	
	执行完就是这样的：![Pasted image 20251005140833.png](<attachments/Pasted image 20251005140833.png>)
	
	这里这个cmd_vel是一个控制的信号（ros侧），也就是ros侧发送控制信号，通过bridge到gazebo侧，然后gazebo里面完成模拟。同时因为有jointstate的插件，gazebo里面也可以计算出joint的状态，然后通过bridg发送回ros。
	
	这里cmd_vel是一个线速度和角速度的向量
	![Pasted image 20251005144234.png](<attachments/Pasted image 20251005144234.png>)
	
	可以简单地通过手动发送这个话题的命令来控制
	ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
	![Pasted image 20251005144313.png](<attachments/Pasted image 20251005144313.png>)
	也可以通过twist控制器来控制
	ros2 run teleop_twist_keyboard teleop_twist_keyboard
	![Pasted image 20251005144343.png](<attachments/Pasted image 20251005144343.png>)


- 6，在gazebo里面创建环境，和保存环境
	![Pasted image 20251005150635.png](<attachments/Pasted image 20251005150635.png>)
	![Pasted image 20251005150706.png](<attachments/Pasted image 20251005150706.png>)
	更多的模型文件在这个网站下载
	https://app.gazebosim.org/fuel/models?page=1&per_page=20
	下载完解压完就是![Pasted image 20251005150825.png](<attachments/Pasted image 20251005150825.png>)
	这个dae就是单独的物体文件
	![Pasted image 20251005150920.png](<attachments/Pasted image 20251005150920.png>)
	然后选择那个下载下来的物体模型就行
	
	*保存世界环境*
	（1）删除机器人！！
	因为我们启动的时候会导入机器人模型，所以世界环境里面不能有机器人模型
	
	（2）![Pasted image 20251005151109.png](<attachments/Pasted image 20251005151109.png>)
	![Pasted image 20251005151145.png](<attachments/Pasted image 20251005151145.png>)
	