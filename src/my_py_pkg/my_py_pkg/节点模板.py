#!/usr/bin/python3 
import rclpy
from rclpy.node import Node 

class MyCustomNode(Node):   #节点名字
    def __init__(self):
        super().__init__("node_name") #节点名字

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() #节点名字
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()