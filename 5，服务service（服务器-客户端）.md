## 服务service

- （1）一种也是用于通讯的方式，
![Pasted image 20250917211636.png](<attachments/Pasted image 20250917211636.png>)
类似于**客户端——服务器**，之间的一种通讯方式
客服端通过**服务名字（网址）**，访问服务器，当然需要传输一些数据称为request
然后服务器计算之后返回给客户端，返回的数据成为response

需要注意的是，服务器和客户端都是由节点写成的。
跟topic话题通讯区别的是，topic通讯是匿名的单向通讯，
而service服务是数据双向传输的通讯
客户端可以是同步的（发出请求之后阻塞直到收到回应）也可以是异步的（发出请求之后该做啥做啥，收到请求之后进行一个回调函数）


- （2）创建一个服务器
	1,最开始关键的是确定数据传输的接口类型：
	![Pasted image 20250917212254.png](<attachments/Pasted image 20250917212254.png>)
	注意”----“，上面是请求request，下面是回应response。这个起的是一个分割作用

	2,写服务器代码（服务器也是由节点构成的）
	/home/hzh/ros2_ws/src/my_py_pkg/my_py_pkg/add_two_ints_server.py
	![Pasted image 20250917212450.png](<attachments/Pasted image 20250917212450.png>)
	需要注意的是，这个【服务名称】，其实有点像topic，可以类比成网址，客户端通过这个网址来访问服务器。所以【服务器名称】是服务器本身节点的名字，【服务名称】是这个服务器所接收服务的代号（名称）


- （3）创建一个客户端（不面向对象型，简单测试）
	/home/hzh/ros2_ws/src/my_py_pkg/my_py_pkg/add_two_ints_client_no_oop.py
	![Pasted image 20250917215525.png](<attachments/Pasted image 20250917215525.png>)


- （4）创建一个客户端（面相对象型）
	/home/hzh/ros2_ws/src/my_py_pkg/my_py_pkg/add_two_ints_client.py
	![Pasted image 20250917221324.png](<attachments/Pasted image 20250917221324.png>)
	
	如果要在客户端的callback里面加入其他参数，得这么加
	![Pasted image 20250917221706.png](<attachments/Pasted image 20250917221706.png>)


- （5）一些常用服务命令
	1，查看一个服务名称对应的接口，详细查看它的数据类型
	![Pasted image 20250921125612.png](<attachments/Pasted image 20250921125612.png>)
	2，简单地向服务访问一下
	![Pasted image 20250918144619.png](<attachments/Pasted image 20250918144619.png>)
	3，通过rqt来简单call一下服务
	![IMG_5802.jpg](attachments/IMG_5802.jpg)
	![Pasted image 20250918145009.png](<attachments/Pasted image 20250918145009.png>)
	
	4，重命名服务名字：（旧服务名字：=新服务名字）
	![Pasted image 20250918145645.png](<attachments/Pasted image 20250918145645.png>)
	![Pasted image 20250918145707.png](<attachments/Pasted image 20250918145707.png>)
	当然节点名字也能重命名，跟以前的方法一样
	![Pasted image 20250918145906.png](<attachments/Pasted image 20250918145906.png>)
	