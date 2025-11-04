#!/usr/bin/python3 
#在同一个节点中定义发布者和订阅者
import rclpy
from rclpy.node import Node 
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        #发布者
        self.number_count_publisher_ = self.create_publisher(Int64,"number_counter",10)
        #订阅者
        self.subscriber_ = self.create_subscription(Int64,"number",self.callback_number,10)
        #服务器
        self.reset_counter_service_ = self.create_service(SetBool,"reset_counter",self.callback_reset_counter)
        self.get_logger().info("NumberCounter has been started.")

    def callback_number(self,msg:Int64):
        # self.get_logger().info(str(msg.data)) #info必须是字符串
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.number_count_publisher_.publish(new_msg)
        
    def callback_reset_counter(self,request:SetBool.Request,response:SetBool.Response):
        if request.data:
            self.counter_ = 0
            response._success = True
            response.message = "Counter has benn reset"
        else:
            response.success = False
            response.message = "Counter has not benn reset"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()