/** 
 *****************************Copyright (c) 2024  ZJU****************************
 * @file      : odometry_node.cpp
 * @brief     : 里程计节点。执行各个传感器节点，并且发布里程计和IMU数据提供给robot_
 * localization 功能包进行扩展卡尔曼滤波
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2024-04-02       Hao Lion        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *                                                                              
 *                                                                              
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2024  ZJU****************************
 */
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "altimeter_node.hpp"
#include "depth_node.hpp"
#include "gps_node.hpp"
#include "imu_node.hpp"

void MySigintHandler(int sig)
{
  (void)sig;
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	rclcpp::shutdown();
    exit(0);
}


int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor   executor;

  auto imu_node = std::make_shared<ImuNode>();
  executor.add_node(imu_node);

  auto depth_node = std::make_shared<DepthNode>();
  executor.add_node(depth_node);


  executor.spin();
  rclcpp::shutdown();

  return 0;
}