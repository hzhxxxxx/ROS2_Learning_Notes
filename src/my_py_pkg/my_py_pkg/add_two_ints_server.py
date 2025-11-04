#!/usr/bin/python3 
#服务器节点
import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):   
    def __init__(self):
        super().__init__("add_two_ints_server") #服务器节点名字
        #创建服务器服务，参数分别是：【接口参数类型】，【服务名称】，【callback函数】
        self.server_ = self.create_service(AddTwoInts,"add_two_ints",self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints server has benn started.")

    def callback_add_two_ints(self,request:AddTwoInts.Request , response:AddTwoInts.Response):
        response.sum = request.a + request.b #具体的request和response，要根据接口的数据类型来决定：ros2 interface show example_interfaces/srv/AddTwoInts 
        self.get_logger().info(str(request.a) + "+" + str(request.b) + "=" + str(response.sum))  
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()