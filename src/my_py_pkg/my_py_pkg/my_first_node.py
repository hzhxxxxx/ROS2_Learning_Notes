#!/usr/bin/python3 
#上面这段代表了默认都用python来编译以下代码，这段路径可以靠图2来查询
import rclpy
from rclpy.node import Node #Node是一个类型

class MyNode(Node):#面向对象编程，
#正常都是直接用上面引用来的Node作为节点的类，
#但是这里我们用子类来继承父类Node，创造一个自己的节点类型MyNode

    def __init__(self):
        super().__init__("py_test") #super继承了父类的默认定义
        #这里的py_test是节点的名字！
        self.counter_ = 0
        self.get_logger().info("hello world")
        #这个是一个播报，类似print。
        self.create_timer(1.0,self.timer_callback)
        #这是一个计时器函数（间隔时间float,执行的事情/函数）

    def timer_callback(self):
        self.get_logger().info(f'hello {str(self.counter_)}')
        self.counter_ +=1

def main(args=None):
    rclpy.init(args=args)#固定格式
    node = MyNode()
    rclpy.spin(node)#让节点保持活着的状态，持续处理回调函数
    rclpy.shutdown()#节点关闭时自动释放空间

if __name__ == '__main__':#相当于固定格式，核心意思是“只有直接运行时才执行的代码“，
    #如果只是被调用，就不执行
    main()