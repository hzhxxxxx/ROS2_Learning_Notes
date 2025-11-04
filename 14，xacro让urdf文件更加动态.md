
 - 1，在urdf文件里面添加xacro声明
	 把文件名后轴加上.xacro，并且在urdf开头的robot定义里面，加入xmlns:xacro="http://www.ros.org/wiki/xacro"
	 ![Pasted image 20251002135025.png](<attachments/Pasted image 20251002135025.png>)


- 2，使用xacro变量（其实叫property）
*注意！使用xacro的变量格式统一为   ${xxx}     ，且变量里面如果有其他数字的话，都要用1位小数！*
	有一些是通用的变量，比如π这种
	![Pasted image 20251002135231.png](<attachments/Pasted image 20251002135231.png>)
	自定义变量   <xacro:property name="base_length" value="0.6" />
	![Pasted image 20251002135304.png](<attachments/Pasted image 20251002135304.png>)


- 3，使用xacro函数（其实叫macro）
	![Pasted image 20251002142524.png](<attachments/Pasted image 20251002142524.png>)
	这里用来创建两个相同的轮子，创建的内容都一样，只是名字不一样，所以这里函数的变量叫做前缀，是一个字符串，到时候直接用${xxx}来引用函数的变量来创建不同名字的轮子


- 4，使用xacro，在一个urdf文件中引用其他urdf文件
	[[12，urdf文件]]

