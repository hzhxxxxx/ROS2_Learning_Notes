#!/usr/bin/python3 
#订阅者
import rclpy
from rclpy.node import Node 
from example_interfaces.msg import String


class SmartphoneNode(Node):

    def __init__(self):
        super().__init__("smartphone")
        self.subscriber_ = self.create_subscription(String,"robot_news",self.callback_robot_news,10)
        #注意接收的数据类型和topic名字必须和之前对应发布者里写的一样！
        #这个第三个参数是，接收这个消息之后干什么，必须是一个函数！
        self.get_logger().info("Smartphone has been started.")

    def callback_robot_news(self,msg:String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()