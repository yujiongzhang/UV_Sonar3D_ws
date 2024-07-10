# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    adis_imu_process = Node(
        package="cirs_girona_cala_viuda",
        executable="adis_imu_process"
    )
    depth_process = Node(
        package="cirs_girona_cala_viuda",
        executable="depth_process"
    )
    dvl_process = Node(
        package="cirs_girona_cala_viuda",
        executable="dvl_process"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [adis_imu_process, depth_process, dvl_process])
    
    # 返回让ROS2根据launch描述执行节点
    return launch_description