#!/usr/bin/python3 
#发布者
import rclpy
from rclpy.node import Node 
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from example_interfaces.msg import Int64
class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher") 
        #创建一个参数 —— “参数名字”，参数默认值（参数的数据类型会根据默认值自动匹配）
        self.declare_parameter("number",2) 
        self.declare_parameter("timer_period",1.0)
        #获取参数的值(注意，直接等于是获取参数的对象，需要.value)
        self.number_ = self.get_parameter("number").value
        self.time_period_ = self.get_parameter("timer_period").value
        #中途参数修改的时候会启动这行，所以需要设定一个函数对应时候启动
        self.add_on_set_parameters_callback(self.parameters_callback)
        


        self.number_publisher_ = self.create_publisher(Int64,"number",10)
        self.number__timer_ = self.create_timer(self.time_period_,self.publish_number)
        self.get_logger().info("Number publisher has been started")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
    
    #参数回调函数
    def parameters_callback(self,params: list[Parameter]):
        #需要返回一个SetParametersResult的对象，其中.successful 代表着修改是否成功
        result = SetParametersResult()
        result.successful = True
        
        #用一个for对每个参数进行修改
        for param in params:
            if param.name == "number":
                self.number_ = param.value
                self.get_logger().info(f"Parameter 'number' changed to: {param.value}")
            else:
                result.successful = False
                result.reason = f"Unknown parameter: {param.name}"
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()