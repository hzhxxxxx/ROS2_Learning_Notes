- 1,创建工作空间：你的 ROS 2 项目的**根目录**
![Pasted image 20250702152743.png](<attachments/Pasted image 20250702152743.png>)

- 2,创建根目录下的专门存放ros2**源**代码的地方
![Pasted image 20250702152927.png](<attachments/Pasted image 20250702152927.png>)

- 3,先创建构建环境
![Pasted image 20250702153107.png](<attachments/Pasted image 20250702153107.png>)
主要是让build，install，log创建出来

- 4,输入bash
![Pasted image 20250702153218.png](<attachments/Pasted image 20250702153218.png>)
![Pasted image 20250702153242.png](<attachments/Pasted image 20250702153242.png>)
(这个意思是提示系统我在这个路径下也有包)

- 5,在源代码空间内创建包
ros2 pkg create --build-type ament_python my_py_pkg
![Pasted image 20250702153544.png](<attachments/Pasted image 20250702153544.png>)

- 6,在源代码空间内进入vs code: 
 ![Pasted image 20250702153654.png](<attachments/Pasted image 20250702153654.png>)

- 7,在创建的包内还有一个名字一样的文件夹，这个文件夹里面才是写代码的地方，创建你的代码文件
![Pasted image 20250702153824.png](<attachments/Pasted image 20250702153824.png>)
![Pasted image 20250702153921.png](<attachments/Pasted image 20250702153921.png>)

(注意，可以加上![Pasted image 20250715152107.png](<attachments/Pasted image 20250715152107.png>)让这个文件变成可执行文件，也就是赋予它权限相当于。可以用ls看到它如果是绿色就说明ok了
)
- 8,写节点代码！
![Pasted image 20250702155249.png](<attachments/Pasted image 20250702155249.png>)
![Pasted image 20250702154225.png](<attachments/Pasted image 20250702154225.png>)

- 9,修改可执行文件
![Pasted image 20250702155919.png](<attachments/Pasted image 20250702155919.png>)

- 10,固定步骤，构建，更新，运行
![Pasted image 20250702155436.png](<attachments/Pasted image 20250702155436.png>)

![Pasted image 20250702155457.png](<attachments/Pasted image 20250702155457.png>)

![Pasted image 20250702155550.png](<attachments/Pasted image 20250702155550.png>)
这里运行的是：ros2 run 包名字 可执行文件的名字
