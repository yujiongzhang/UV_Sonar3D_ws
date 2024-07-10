#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
# @Time    : 2024.1.10
# @Author  : haolion520@gmail.com
# @File    : dvl_node.py
"""

# ros2库
import rclpy # ros2 python接口库
from rclpy.node import Node # ros2 节点类 
import socket
import json
from time import sleep
from geometry_msgs.msg import TwistWithCovarianceStamped


oldJson = ""

'''try to connect dvl'''
def connect(sockfd,ip_address, port):
    try:
        sockfd.connect((ip_address, port))
        sockfd.settimeout(1)
    except socket.error as err:
        print("No route to host, DVL might be booting? {}".format(err))
        sleep(1)
        connect(sockfd,ip_address, port)

'''get raw json data'''
def get_data(sockfd,ip_address, port):
    global oldJson
    raw_data = ''
    while not '\n' in raw_data:
        try:
            rec = sockfd.recv(1).decode('utf-8')
            if len(rec) == 0:
                print("Socket closed by the DVL, reopening")
                connect(sockfd, ip_address, port)
                continue
        except socket.timeout as err:
            print("Lost connection with the DVL, reinitiating the connection: {}".format(err))
            connect(sockfd, ip_address, port)
            continue
        raw_data = raw_data + rec
    
    raw_data = oldJson + raw_data
    
    raw_data = raw_data.split('\n')
    oldJson = ""
    oldJson = raw_data[1]
    raw_data = raw_data[0]
    
    return raw_data

""" parse json data """
def parse_data(raw_data):
    try:
        data = json.loads(raw_data)
        if data["type"] == "velocity":
            
            velocitydict = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
            covariance = data["covariance"]
            velocitydict['vx'] = data["vx"]
            velocitydict['vy'] = data["vy"]
            velocitydict['vz'] = data["vz"]
            
            return velocitydict, covariance
    except json.JSONDecodeError as e:
        # 处理解码错误，例如记录错误、尝试其他编码、跳过此消息等
        print("Error decoding message payload. Payload may not be a UTF-8 string.")

    return None, None

class DvlNode(Node):

    """construct function"""
    def __init__(self):
        super().__init__('dvl_node') # ros2节点父类初始化,节点名称为capture_image

        self.declare_parameter('ip', "192.168.1.95")  
        self.dvl_ip = self.get_parameter('ip').get_parameter_value().string_value

        self.declare_parameter('port', 16171) 
        self.dvl_port = self.get_parameter('port').get_parameter_value().integer_value  

        self.pub = self.create_publisher(TwistWithCovarianceStamped, 'dvl', 10)


    def publisher(self, velocitydict, covariance):
        dvl_msg = TwistWithCovarianceStamped()
        dvl_msg.header.stamp = self.get_clock().now().to_msg()
        dvl_msg.header.frame_id = "base_link"
        
        dvl_msg.twist.twist.linear.x = float(velocitydict["vx"])
        dvl_msg.twist.twist.linear.y = float(velocitydict["vy"])
        dvl_msg.twist.twist.linear.z = float(velocitydict["vz"])
        dvl_msg.twist.covariance[0] = covariance[0][0];
        dvl_msg.twist.covariance[1] = covariance[0][1];
        dvl_msg.twist.covariance[2] = covariance[0][2];

        dvl_msg.twist.covariance[3] = covariance[1][0];
        dvl_msg.twist.covariance[4] = covariance[1][1];
        dvl_msg.twist.covariance[5] = covariance[1][2];

        dvl_msg.twist.covariance[6] = covariance[2][0];
        dvl_msg.twist.covariance[7] = covariance[2][1];
        dvl_msg.twist.covariance[8] = covariance[2][2];
        
        self.pub.publish(dvl_msg)


    
def main(args=None): # ros2 节点主入口main函数
    rclpy.init(args=args) # ros2 python接口初始化

    node = DvlNode() # 创建ros2节点对象并进行初始化

    sockfd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    connect(sockfd ,node.dvl_ip, node.dvl_port)

    while rclpy.ok():
        data = get_data(sockfd ,node.dvl_ip, node.dvl_port)
        velocitydict,covariance = parse_data(data)
        if velocitydict != None and covariance != None:
            node.publisher(velocitydict,covariance)

    sockfd.close()
    node.destroy_node() # 销毁节点对象
    rclpy.shutdown() # 关闭ros2 python接口

if __name__ == "__main__":
    main()
