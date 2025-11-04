#!/usr/bin/python3 
#客户端节点
from urllib import response
import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):   
    def __init__(self):
        super().__init__("add_two_ints_client") 
        self.client_ = self.create_client(AddTwoInts,"add_two_ints")

    def call_add_two_ints(self,a,b):
        #等待跟服务器连接上
        while not self.client_.wait_for_service(1.0):#超时时间，其实可以理解为每隔多少s访问一次服务器，这句话才真正开始连接服务器！
            self.get_logger().info("Waiting for Add Two Ints server...")
        #创建一个请求
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        #异步发送请求
        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_call_add_two_ints)

    def callback_call_add_two_ints(self,future):#收到回应之后做什么
        response = future.result()
        self.get_logger().info("Got response: " + str(response.sum))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient() 
    node.call_add_two_ints(2,7)#产生请求的函数
    node.call_add_two_ints(1,7)
    node.call_add_two_ints(10,20)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()