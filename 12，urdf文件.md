URDF就是机器人的"完整描述文件"
类比于solidworks里面的装配体
包含：
（1），三维几何信息
（2），坐标系关系（TF 树结构）
（3），一些物理属性
	质量、惯性矩阵
	关节的运动限制（角度范围、速度限制等）


- 1,创建一个urdf文件
![Pasted image 20250929214943.png](<attachments/Pasted image 20250929214943.png>)

- 2,开始编写urdf文件
![Pasted image 20250929215858.png](<attachments/Pasted image 20250929215858.png>)
注意，存在两个以上link的时候必须加入tf，因为一个urdf文件只能有一个根节点！
或者说全局只能有一个无父母link！

- 3,运行rivz
![Pasted image 20250929220039.png](<attachments/Pasted image 20250929220039.png>)
后面的 model:= 就是这个urdf文件的路径


### 不同link部件形状：
- box：长方体
- cylinder：圆柱体
- sphere：球体
- mesh：自定义的三维体（SolidWorks导出）
	![Pasted image 20251002152024.png](<attachments/Pasted image 20251002152024.png>)
	mesh filename="package://。。。。。.stl 文件路径" scale="0.001 0.001 0.001"(这个是缩放，SolidWorks里面一般用mm，而我们这里一般用m，所以需要缩小1000倍，要不直接放进来的话太大了)


### 不同关节joint类型：

- fixed：一种只表示连接，不能运动的关节（如上图）
- revolute：有限度的旋转关节
	![Pasted image 20251001125411.png](<attachments/Pasted image 20251001125411.png>)
- continuous：无限度的旋转关节（例如车轮）
	![Pasted image 20251001125612.png](<attachments/Pasted image 20251001125612.png>)
- prismatic：直线的滑动关节
	![Pasted image 20251001130130.png](<attachments/Pasted image 20251001130130.png>)



(注意，在rviz里面可以通过global options-fixed frame来设定，哪个坐标系与世界坐标系原点对齐)
![Pasted image 20251001135938.png](<attachments/Pasted image 20251001135938.png>)


### 创建一个实际的urdf包：

- 1，在工作区—代码区里面，创建一个description的包，然后在里面移除include/ 和 src/这两个文件夹，并且在里面创建一个文件夹叫urdf，在urdf文件夹里面放入我们之前写好的urdf文件
![Pasted image 20251001155119.png](<attachments/Pasted image 20251001155119.png>)
![Pasted image 20251001155151.png](<attachments/Pasted image 20251001155151.png>)

- 2，修改cmake
![Pasted image 20251001155421.png](<attachments/Pasted image 20251001155421.png>)

- 3，colcon build一下就行了


### 在一个urdf文件中引用其他urdf文件（xacro）

- 对于被他人引用的子文件，不需要robot的name，但是xacro的声明还是要的
![Pasted image 20251002145515.png](<attachments/Pasted image 20251002145515.png>)
![Pasted image 20251002145556.png](<attachments/Pasted image 20251002145556.png>)
注意，这里第二个子文件创建link的时候，其实用到了子文件1中的材料，所以在主程序中，最好先引用子文件1，再引用子文件2（其实放在后面也行，因为他这个引用程序的逻辑其实是分为两个阶段，第一阶段先把包含的所以内容收集起来然后展开，第二阶段再根据展开的内容执行，所以其实也能读取到，只是为了代码的可读性，最好按顺序摆放）

主程序：launch启动文件输入的依然是这个主程序，只是它把它的内容拆成了好几个子urdf文件
![Pasted image 20251002145712.png](<attachments/Pasted image 20251002145712.png>)
<xacro:include filename="common_properties.urdf.xacro" />


### 转动惯量矩阵

转动惯量本身相当于旋转运动中的质量
转动惯量与质量和尺寸有关（当然这里为了转动惯量和物体模型匹配，所以原点也要一样）

这里用的是通过设置一个形状的通用转动惯量作为函数，然后这样同一个形状的link就可以直接调用这个函数来设置自己的转动惯量
![Pasted image 20251004133052.png](<attachments/Pasted image 20251004133052.png>)
这个转动惯量矩阵提一嘴：
这是一个 3×3 的对称矩阵：
  | ixx  ixy  ixz |
  | ixy  iyy  iyz |
  | ixz  iyz  izz |
  所以其实只需要写入对角线右上角的部分，也就是
  xx xy xz
	yy yz
	   zz
各个元素含义：
  - ixx, iyy, izz：绕 X、Y、Z 轴的转动惯量（主对角线）
  - ixy, ixz, iyz：交叉项，表示不同轴之间的耦合
（但是一般只需要考虑xx，yy，zz，其它项大部分情况下都是0）

关于转动惯量矩阵，详见：
https://en.wikipedia.org/wiki/List_of_moments_of_inertia

在link里面定义的时候，直接调用函数就行
![Pasted image 20251004133533.png](<attachments/Pasted image 20251004133533.png>)
注意就是xyz和原点都是跟link的设置数值一模一样的


在rviz里面，把link可视化关闭，tf关闭之后，打开这个就能显示现在的转动惯量模型，应该是跟物体模型匹配才对（注意，他这个转动惯量的模型全部都会显示成长方体，所以即使是圆柱体或者球体的转动惯量在rviz里面也会显示得形状不一样，不过没关系，位置一样，然后形状大小不差太多就可以的）
![Pasted image 20251004133656.png](<attachments/Pasted image 20251004133656.png>)


### 碰撞设置

其实就是要设置一下这个link的碰撞范围（一般用实心体设置），否则就会不跟地面碰撞导致直接沉入地下，或是因为没有碰撞导致穿过障碍物
![Pasted image 20251004143650.png](<attachments/Pasted image 20251004143650.png>)
其实碰撞设置里面的内容跟可视化的内容差不多，就是没有material。

注意，对于圆柱体来说，最好碰撞体设置成球体！
![Pasted image 20251004143752.png](<attachments/Pasted image 20251004143752.png>)

rviz里面这个设置可以看到碰撞体的范围
![Pasted image 20251004143843.png](<attachments/Pasted image 20251004143843.png>)


### 在urdf里面插入gazebo插件

其实加入gazebo插件之后，就是在gazebo侧，会由于这些插件，他们会发布一些特定的话题

（1），在主urdf里面加入一个新文件![Pasted image 20251005134303.png](<attachments/Pasted image 20251005134303.png>)

（2），在新文件里面写入gazebo plugin插件

![Pasted image 20251005134431.png](<attachments/Pasted image 20251005134431.png>)

详见https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems
![Pasted image 20251005134455.png](<attachments/Pasted image 20251005134455.png>)
这里面全是各种插件

比如我们这里装的diffdrive
*那么插件的filename的规则就是gz-sim-插件名字-system（按单词分词，比如这里DiffDrive，就是diff-drive）*
![Pasted image 20251005134600.png](<attachments/Pasted image 20251005134600.png>)
*这个。cc属于源文件，插件的name名字在里面拉到最后找到*
![Pasted image 20251005134719.png](<attachments/Pasted image 20251005134719.png>)
*然后。hh属于说明文件，这里面全是这个插件你需要填写的参数*
![Pasted image 20251005134922.png](<attachments/Pasted image 20251005134922.png>)

