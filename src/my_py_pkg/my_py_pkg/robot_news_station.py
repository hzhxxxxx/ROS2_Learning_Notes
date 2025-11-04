#!/usr/bin/python3 
#发布者
import rclpy
from rclpy.node import Node 
from example_interfaces.msg import String
#example_interfaces这是ROS 2提供的一个标准接口包,
#在其的msg里面有很多系统自带定义的消息类型
#注意这里引入了新的依赖库，那么我们就得在“package.xml"里面加上新的依赖库名字  <depend>example_interfaces</depend>

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station") 
        self.declare_parameter("robot_name","C3PO")
        self.robot_name_ = self.get_parameter("robot_name").value
        self.publisher_= self.create_publisher(String,"robot_news",10)
        #这里是创建一个“发布者”！
        #数据类型，topic名字！，10
        self.timers_ = self.create_timer(0.5,self.publish_news)#每0.5s触发一次publish——news函数
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self):#这个函数代表的才是调用创建的发布者来发布具体的消息！
        msg = String()
        #这里这个String其实是一个字符串的数据类型的类
        #这里通过这个类创建一个msg的对象用来表示消息。
        #而String这个类里面包含这两个信息：1,string表示具体数据类型的 2,data这个表示消息的具体内容
        msg.data = f"hello,this is {self.robot_name_} from the robot news station"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()