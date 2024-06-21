# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    m750d_pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        parameters = [{
            # 'target_frame': 'laser_link',
            'transform_tolerance':0.01,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -1.134464, # -65
            'angle_max':  1.134464, # 65
            'angle_increment': 0.008863,
            'scan_time' : 0.1, # 10hz
            'range_min': 0.5,
            'range_max': 20.0
        }],
        remappings=[
            ('cloud_in', '/points2'),
            ('scan','/sonar/scan')
        ]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [m750d_pointcloud_to_scan])
    # 返回让ROS2根据launch描述执行节点
    return launch_description