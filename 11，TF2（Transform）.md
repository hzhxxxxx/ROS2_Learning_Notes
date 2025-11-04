*TF2 维护一个树形结构——tf tree
每个节点就是这个机器人的每个部件的坐标系
然后连接的方式就是用一个变换矩阵来连接*

*这样就记录了机器人各个部件之间的相对位置关系*


下面是简单的看一下rviz和tf：

安装需要的东西：
![Pasted image 20250929202337.png](<attachments/Pasted image 20250929202337.png>)
![Pasted image 20250929202357.png](<attachments/Pasted image 20250929202357.png>)

进入这个文件夹（里面放着很多示例的urdf文件），打开rviz
![Pasted image 20250929202454.png](<attachments/Pasted image 20250929202454.png>)
hzh@hzh-Lenovo-Legion-R7000P2021:/opt/ros/humble/share/urdf_tutorial/urdf$ ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/humble/share/urdf_tutorial/urdf/08-macroed.urdf.xacro

rviz
![Pasted image 20250929202615.png](<attachments/Pasted image 20250929202615.png>)
左边是控制一些关节的运动
右边是主界面
robotmodel——links：是各个部件
tf——frames：是各个坐标系
tf——tree：是tf树

这个tf树还可以直接从可视化里面看到（把部件都隐藏了更容易发现）
![Pasted image 20250929202842.png](<attachments/Pasted image 20250929202842.png>)
这些箭头代表了父子关系

还可以![Pasted image 20250929202937.png](<attachments/Pasted image 20250929202937.png>)
他会在当前路径下生成两个文件
![Pasted image 20250929203005.png](<attachments/Pasted image 20250929203005.png>)
其中一个是tf树的pdf
![Pasted image 20250929203036.png](<attachments/Pasted image 20250929203036.png>)

注意机器人所有的坐标都会汇集到一个树头节点上面叫做base
*tf的树形结构也解释了，移动腿，脚的坐标系也会跟着动的原因（因为脚是腿的子叶）*



