#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "oculus_interfaces/msg/ping.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>

#include <cmath>



class m750d_ping2pc : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr m750d_ping_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m750d_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m750d_edge_pointcloud_publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m750d_laserscan_publisher_;

public:
    // 构造函数,有一个参数为节点名称
    m750d_ping2pc(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        m750d_ping_subscribe_ = this->create_subscription<oculus_interfaces::msg::Ping>("/sonar/ping", 10, \
                                    std::bind(&m750d_ping2pc::m750d_ping_callback, this, std::placeholders::_1));
        m750d_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/points2", 10); 
        m750d_edge_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/sonar/edge_points", 10); 
        // m750d_laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/m750d/scan", 10);
    }

    // 收到 m750d ping 的回调函数
    void m750d_ping_callback(const oculus_interfaces::msg::Ping::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "receive /sonar/ping");
        const int n_beams = msg->n_beams;
        int n_ranges = msg -> n_ranges;
        int n_step = msg -> step;
        double scanRange = msg->range;
        int offset = 2048;

        std::vector<int16_t> bearings(n_beams);
        memcpy((void*)bearings.data(), (void*)(msg->bearings).data(), n_beams*sizeof(int16_t));
        std::vector<std::vector<uint8_t>> range_beam_map;
        std::vector<uint8_t> one_range(n_beams);
        for(int i = 0; i<n_ranges;i++){
            memcpy((void*)one_range.data(), (void*)((msg->ping_data).data()+ offset + 4 +n_step*i ), n_beams);
            range_beam_map.push_back(one_range);
        }

        std::vector<std::pair<double, double>> points_map;//存储符合要求的点的map
        std::vector<std::pair<double, double>> edge_points_map;//存储符合要求的边缘点的map

        for (int beams_i = 0; beams_i < n_beams; beams_i++)
        {
            bool is_edge = true;
            for (int range_i = 0; range_i < n_ranges; range_i++)
            {
                if (range_beam_map[range_i][beams_i]>110) // 能量阈值设置
                {
                    double angle = (float)bearings[beams_i]/18000.0 * M_PI;
                    double range = ((float)range_i / (float)n_ranges) * scanRange;

                    double px = cos(angle) * range;
                    double py = sin(angle) * range;
                    points_map.push_back(std::pair<double, double>(px, py));
                    if (is_edge && range > 1.5)
                    {
                        edge_points_map.push_back(std::pair<double, double>(px, py));
                        is_edge = false;
                    }
                }
            }
        }

        sensor_msgs::msg::PointCloud2 pc2_msg;
        pc2_msg.header = msg->header;
        pc2_msg.header.frame_id = "laser_link";
        
        pc2_msg.height = 1;
        pc2_msg.width = points_map.size();
        pc2_msg.fields.resize(3);
        pc2_msg.fields[0].name = "x";
        pc2_msg.fields[0].offset = 0;
        pc2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2_msg.fields[0].count = 1;
        pc2_msg.fields[1].name = "y";
        pc2_msg.fields[1].offset = 4;
        pc2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2_msg.fields[1].count = 1;
        pc2_msg.fields[2].name = "z";
        pc2_msg.fields[2].offset = 8;
        pc2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2_msg.fields[2].count = 1;
        // pc2_msg.fields[3].name = "intensity";
        // pc2_msg.fields[3].offset = 12;
        // pc2_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        // pc2_msg.fields[3].count = 1;

        pc2_msg.point_step = 12;
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width;
        pc2_msg.is_bigendian = false;
        pc2_msg.is_dense = true;
        pc2_msg.data.resize(pc2_msg.width * pc2_msg.point_step);

        float* data_ptr = reinterpret_cast<float*>(&pc2_msg.data[0]);

        for (int i=0; i<(int) points_map.size(); i++) {
            int index = i * 3;
            data_ptr[index] = points_map[i].first;
            data_ptr[index+1] = points_map[i].second;
            data_ptr[index+2] = 0.0;
        }


        sensor_msgs::msg::PointCloud2 edge_pc2_msg;//边缘点
        edge_pc2_msg.header = msg->header;
        edge_pc2_msg.header.frame_id = "laser_link";
        
        edge_pc2_msg.height = 1;
        edge_pc2_msg.width = edge_points_map.size();
        edge_pc2_msg.fields.resize(3);
        edge_pc2_msg.fields[0].name = "x";
        edge_pc2_msg.fields[0].offset = 0;
        edge_pc2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        edge_pc2_msg.fields[0].count = 1;
        edge_pc2_msg.fields[1].name = "y";
        edge_pc2_msg.fields[1].offset = 4;
        edge_pc2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        edge_pc2_msg.fields[1].count = 1;
        edge_pc2_msg.fields[2].name = "z";
        edge_pc2_msg.fields[2].offset = 8;
        edge_pc2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        edge_pc2_msg.fields[2].count = 1;
        // pc2_msg.fields[3].name = "intensity";
        // pc2_msg.fields[3].offset = 12;
        // pc2_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        // pc2_msg.fields[3].count = 1;

        edge_pc2_msg.point_step = 12;
        edge_pc2_msg.row_step = edge_pc2_msg.point_step * edge_pc2_msg.width;
        edge_pc2_msg.is_bigendian = false;
        edge_pc2_msg.is_dense = true;
        edge_pc2_msg.data.resize(edge_pc2_msg.width * edge_pc2_msg.point_step);

        float* edge_data_ptr = reinterpret_cast<float*>(&edge_pc2_msg.data[0]);

        for (int i=0; i<(int) edge_points_map.size(); i++) {
            int index = i * 3;
            edge_data_ptr[index] = edge_points_map[i].first;
            edge_data_ptr[index+1] = edge_points_map[i].second;
            edge_data_ptr[index+2] = 0.0;
        }

        m750d_pointcloud_publisher_->publish(pc2_msg);
        m750d_edge_pointcloud_publisher_->publish(edge_pc2_msg);

        RCLCPP_INFO(this->get_logger(), "beams: %d ,step: %d",n_beams, n_step);
        RCLCPP_INFO(this->get_logger(), "scanRange: %f",scanRange);
        RCLCPP_INFO(this->get_logger(), "bearings: %d %d %d ... %d %d %d",\
                    bearings[0],bearings[1], bearings[2],bearings[n_beams - 3],bearings[n_beams - 2], bearings[n_beams - 1]);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<m750d_ping2pc>("m750d_ping2pc");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

