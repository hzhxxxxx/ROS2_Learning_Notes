目的是，用一个启动文件来同时启动多个节点

【1】，创建放启动文件的文件夹
跟[[6，自定义接口interface]]的前几步有点像

在工作区的代码区内建立我们的启动文件夹包
![Pasted image 20250926153629.png](<attachments/Pasted image 20250926153629.png>)


【2】，进入我们创建的这个文件夹中，然后删掉include和src这两个文件夹，只留CMakeList和package
![Pasted image 20250926153710.png](<attachments/Pasted image 20250926153710.png>)

【3】，在这个文件夹内，创建一个文件夹叫“launch”，在launch文件夹下创建一个启动文件：名字.launch.xml
![Pasted image 20250926154011.png](<attachments/Pasted image 20250926154011.png>)
![Pasted image 20250926154105.png](<attachments/Pasted image 20250926154105.png>)



【4】，在package.xml里面只需要加入你要运行的节点的包名字作为依赖就行

（强烈建议在写启动文件之前先一个个用ros2 run格式看一下每个文件的”包名字“和“可执行文件名字”）
![Pasted image 20250926154225.png](<attachments/Pasted image 20250926154225.png>)
![Pasted image 20250926154300.png](<attachments/Pasted image 20250926154300.png>)
注意是<exec_depend>

<exec_depend>my_py_pkg</exec_depend>


【5】，在Cmake中编辑，把多余内容删掉，加入这部分：
![Pasted image 20250926154406.png](<attachments/Pasted image 20250926154406.png>)
install(DIRECTORY

launch

DESTINATION share/${PROJECT_NAME}/

)

(注意要大写)


【6】，写启动文件.launch.xml
![Pasted image 20250926154651.png](<attachments/Pasted image 20250926154651.png>)


【7】，构建一下这个启动文件的包
![Pasted image 20250926154738.png](<attachments/Pasted image 20250926154738.png>)

【8】，用ros2 launch +启动文件包名字 +启动文件名字
同时启动多个节点
![Pasted image 20250926154808.png](<attachments/Pasted image 20250926154808.png>)


【9】，*在启动文件中重新映射节点名字和话题名字*

以前重新映射这两个是这么写![Pasted image 20250722104113.png](<attachments/Pasted image 20250722104113.png>)

现在是
![Pasted image 20250926162806.png](<attachments/Pasted image 20250926162806.png>)
< remap from="/number" to="/my_number" / >

【10】，*在启动文件中写入参数*

以前写入参数是这样![Pasted image 20250924190116.png](<attachments/Pasted image 20250924190116.png>)

现在是
![Pasted image 20250926165020.png](<attachments/Pasted image 20250926165020.png>)
< param name="number" value="6" / >

或者通过yaml文件来写入参数
我们之前创建的yaml文件夹在主项目文件夹的外面，这次我们把他放进我们这个bringup包里面

先在bringup下面创建一个放yaml文件的文件夹
![Pasted image 20250926165318.png](<attachments/Pasted image 20250926165318.png>)

创建yaml文件
![Pasted image 20250926165425.png](<attachments/Pasted image 20250926165425.png>)

注意，因为启动文件要用到放yaml的文件夹，所以要在Cmake里面加入那个文件夹的名字！
![Pasted image 20250926165548.png](<attachments/Pasted image 20250926165548.png>)

在启动文件中，通过yaml文件来写入参数
![Pasted image 20250926165749.png](<attachments/Pasted image 20250926165749.png>)< param from="$(find-pkg-share my_robot_bringup)/config/number_app.yaml" / >