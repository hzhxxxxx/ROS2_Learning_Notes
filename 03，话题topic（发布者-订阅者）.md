
### topic的定义：

用广播电台作比喻
![Pasted image 20250715151346.png](<attachments/Pasted image 20250715151346.png>)
topic有几个特点：
- 1,数据单向传播
- 2,匿名（不管是发布者还是订阅者）
- 3,topic有一个数据类型！所有在这个topic上的发布者和订阅者必须都使用与该topic相对应的数据类型！（就像电台发出的广播如果是fm类型，你就必须用fm才能解码订阅该广播）


### 创建发布者节点

跟之前创建节点类似
- 1，一开始创建文件![Pasted image 20250715155431.png](<attachments/Pasted image 20250715155431.png>)

- 2,写代码
![Pasted image 20250715155516.png](<attachments/Pasted image 20250715155516.png>)

写代码过程中注意，增加依赖库：![Pasted image 20250715155620.png](<attachments/Pasted image 20250715155620.png>)

增加可执行文件名字：![Pasted image 20250715155702.png](<attachments/Pasted image 20250715155702.png>)

- 3,然后一样的构建包，更新，运行

![Pasted image 20250715155754.png](<attachments/Pasted image 20250715155754.png>)
![Pasted image 20250715155803.png](<attachments/Pasted image 20250715155803.png>)
![Pasted image 20250715155809.png](<attachments/Pasted image 20250715155809.png>)


然后我们可以用node list看到现在正在运行的节点
![Pasted image 20250715155834.png](<attachments/Pasted image 20250715155834.png>)

也可以看到现在在场上的topic有哪些
![Pasted image 20250715160114.png](<attachments/Pasted image 20250715160114.png>)

然后可以指定一个topic在命令行中暂时地接收查看：
![Pasted image 20250715160201.png](<attachments/Pasted image 20250715160201.png>)

可以查看一个topic的信息，包括它的数据类型，发布者数量和接受者数量
![Pasted image 20250722102536.png](<attachments/Pasted image 20250722102536.png>)

可以观察一个topic发布的频率 
![Pasted image 20250722102948.png](<attachments/Pasted image 20250722102948.png>)

同样可以观察一个topic发布信息的内容大小
![Pasted image 20250722103140.png](<attachments/Pasted image 20250722103140.png>)

给这个topic加入新的发布内容，5是hz
![Pasted image 20250722103614.png](<attachments/Pasted image 20250722103614.png>)


- 重新映射名字

只改发布者的节点名字，topic名字不变
![Pasted image 20250722103913.png](<attachments/Pasted image 20250722103913.png>)

还想要改topic的名字就要再-r一次，这里后面的robot_news是原来topic的名字
![Pasted image 20250722104113.png](<attachments/Pasted image 20250722104113.png>)

同样也可以改订阅者的订阅topic名字
![Pasted image 20250722104338.png](<attachments/Pasted image 20250722104338.png>)




### 创建订阅者节点

- 1,创建文件跟之前一样
![Pasted image 20250715162005.png](<attachments/Pasted image 20250715162005.png>)

- 2，写代码
![Pasted image 20250715162035.png](<attachments/Pasted image 20250715162035.png>)

依旧检查注意，依赖库有没有新增，然后加入新的可执行文件名字

- 3,然后一样的构建包，更新，运行
![Pasted image 20250715162135.png](<attachments/Pasted image 20250715162135.png>)
![Pasted image 20250715162202.png](<attachments/Pasted image 20250715162202.png>)
![Pasted image 20250715162210.png](<attachments/Pasted image 20250715162210.png>)


此时同时运行发布者和订阅者，rqt_graph就会
![Pasted image 20250715162305.png](<attachments/Pasted image 20250715162305.png>)