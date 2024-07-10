#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
# @Time    : 2024.4.10
# @Author  : haolion520@gmail.com
# @File    : ugps_node.py
# @berif   : Get position from Water Linked Underwater GPS
"""


# ros2库
import rclpy # ros2 python接口库
from rclpy.node import Node # ros2 节点类 
import argparse
import json
import requests
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

def get_data(url):
    try:
        r = requests.get(url)
    except requests.exceptions.RequestException as exc:
        print("Exception occured {}".format(exc))
        return None

    if r.status_code != requests.codes.ok:
        print("Got error {}: {}".format(r.status_code, r.text))
        return None

    return r.json()

def get_antenna_position(base_url):
    return get_data("{}/api/v1/config/antenna".format(base_url))

def get_acoustic_position(base_url):
    return get_data("{}/api/v1/position/acoustic/filtered".format(base_url))

def get_global_position(base_url, acoustic_depth = None):
    return get_data("{}/api/v1/position/global".format(base_url))

class UgpsNode(Node):

    """construct function"""
    def __init__(self):
        super().__init__('ugps_node') # ros2节点父类初始化,节点名称为ugps_node

        self.declare_parameter('url', "https://demo.waterlinked.com")  
        self.ugps_url = self.get_parameter('url').get_parameter_value().string_value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'ugps', 10)

        self.timer = self.create_timer(0.5, self.time_callback)

    def time_callback(self):
        acoustic_position = get_acoustic_position(self.ugps_url)
        if acoustic_position:
            # print("Current acoustic position. X: {}, Y: {}, Z: {}".format(
            #         acoustic_position["x"],
            #         acoustic_position["y"],
            #         acoustic_position["z"]))
            ugps_msg = PoseWithCovarianceStamped()
            ugps_msg.header.stamp = self.get_clock().now().to_msg()
            ugps_msg.header.frame_id = "base_link"
            ugps_msg.pose.pose.position.x = float(acoustic_position["x"])
            ugps_msg.pose.pose.position.y = float(acoustic_position["y"])
            # ugps_msg.pose.pose.position.z = acoustic_position["z"]        #深度方向由深度计校准
            ugps_msg.pose.covariance = list(np.float_([
                0.0002, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0002, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0
            ]))
            self.pub.publish(ugps_msg)

def main(args=None): # ros2 节点主入口main函数
    rclpy.init(args=args) # ros2 python接口初始化
    
    rclpy.spin(UgpsNode())

    rclpy.shutdown() # 关闭ros2 python接口

if __name__ == "__main__":
    main()
            

