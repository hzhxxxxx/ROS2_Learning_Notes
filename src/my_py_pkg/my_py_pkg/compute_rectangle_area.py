#!/usr/bin/python3 
#使用自定义接口的服务器
from urllib import request
import rclpy
from rclpy.node import Node 
from my_robot_interfaces.srv import ComputeRectangleArea

class ComputeRectangleAreaNode(Node):   
    def __init__(self):
        super().__init__("compute_rectangle_area_server") 
        self.compute_rectangle_area_server_ = self.create_service(ComputeRectangleArea,"compute_rectangle_area",self.callback_compute_rectangle_area)
        self.get_logger().info("ComputeRectangleArea Node has benn started.")

    def callback_compute_rectangle_area(self, request:ComputeRectangleArea.Request, response:ComputeRectangleArea.Response):
        response.area = request.length * request.width
        self.get_logger().info("area" + "=" + str(response.area))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeRectangleAreaNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()