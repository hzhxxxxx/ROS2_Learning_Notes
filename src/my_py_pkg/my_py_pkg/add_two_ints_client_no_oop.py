#!/usr/bin/python3 
from asyncio import wait_for
from urllib import request, response
import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts


def main(args=None):
    #初始化节点
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")
    #初始化客户端，并且等待跟服务器连接上
    client = node.create_client(AddTwoInts,"add_two_ints")
    while not client.wait_for_service(1.0):#超时时间，其实可以理解为每隔多少s访问一次服务器
        node.get_logger().info("Waiting for Add Two Ints server...")
    #创建一个请求
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    future = client.call_async(request)#异步发送请求，最好用异步，同步的话这里会出现问题
    rclpy.spin_until_future_complete(node,future)#一直运行节点直到收到future

    response = future.result()#收到返回信号
    node.get_logger().info(str(request.a) + "+" + str(request.b) + "=" + str(response.sum))

    rclpy.shutdown()

if __name__ == '__main__':
    main()