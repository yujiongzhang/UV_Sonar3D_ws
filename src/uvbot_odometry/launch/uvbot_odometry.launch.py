# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os.path

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="YellowBot",
            description="Top-level namespace",
        )
    )

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")

    pkg_name = 'uvbot_odometry'

    sensor_param = os.path.join(get_package_share_directory(pkg_name), 'config', 'sensor_config.yaml')

    print(sensor_param)

    serial_sensor_node = Node(
        package=pkg_name,
        namespace= namespace,
        executable="odometry_node",
        parameters=[sensor_param],
        output="both",
    )

    dvl_node = Node(
        package=pkg_name,
        namespace=namespace,
        executable="dvl_node",
        parameters=[{
            'ip': "192.168.1.95",
            'port': 16171,
        }]
    )

    ugps_node = Node(
        package=pkg_name,
        namespace=namespace,
        executable="ugps_node",
        parameters=[{
            'url': "https://demo.waterlinked.com", #https://demo.waterlinked.com
        }]
    )

    ekf_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'odometry_zyj.yaml')

    ekf_node = Node(
        package='robot_localization',
        namespace=namespace,
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    nodes = [
        # serial_sensor_node,
        # dvl_node,
        ekf_node
    ]

    return LaunchDescription(declared_arguments + nodes)
