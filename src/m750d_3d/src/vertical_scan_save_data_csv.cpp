#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"

#include "cv_bridge/cv_bridge.h"
#include <pcl_conversions/pcl_conversions.h>

#include "opencv2/opencv.hpp"
#include "image_fun.h"

#include <time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "oculus_interfaces/msg/ping.hpp"

#include <chrono>
#include <cmath>
#include <fstream>

// 生成当前时间的字符串，格式为 YYYYMMDD_HHMMSS
std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}


void saveVectorToCSV(const std::vector<std::vector<uint8_t>>& data, const std::string& filename) {
    // 打开文件流
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 遍历二维向量
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << static_cast<int>(row[i]);  // 将uint8_t转换为int写入文件
            if (i < row.size() - 1) {
                file << ",";  // 用逗号分隔每个元素
            }
        }
        file << "\n";  // 换行
    }

    // 关闭文件流
    file.close();
    std::cout << "文件已保存到: " << filename << std::endl;
}

void saveVectorToBeams(const std::vector<int16_t>& data, const std::string& filename) {
    // 打开文件流
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 遍历向量，将每个元素写入文件
    for (const auto& value : data) {
        file << value << "\n";  // 每个值占一行
    }

    // 关闭文件流
    file.close();
    std::cout << "文件已保存到: " << filename << std::endl;
}

void saveVectorToDepths(const std::vector<float>& data, const std::string& filename) {
    // 打开文件流
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 遍历向量，将每个元素写入文件
    for (const auto& value : data) {
        file << value << "\n";  // 每个值占一行
    }

    // 关闭文件流
    file.close();
    std::cout << "文件已保存到: " << filename << std::endl;
}

class vertical_scan : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_subscription_;
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr m750d_ping_subscribe_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;

    double ScanRange;
    double PixelResolution;
    double z_p;
    volatile float current_depth;
    // 生成带时间戳的文件名
    std::string folder_name;
    int index;
    std::vector<float> depth_vector;

public:
    vertical_scan(std::string name) : Node(name)
    {
        folder_name = getCurrentTimeString();
        system(("mkdir "+folder_name).c_str());

        index = 1;
        pointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        ScanRange = 10.0;
        z_p = 0;
        
        m750d_ping_subscribe_ = this->create_subscription<oculus_interfaces::msg::Ping>("/sonar/ping", 10, \
            std::bind(&vertical_scan::m750d_ping_callback, this, std::placeholders::_1));

        // 创建一个具有 Best Effort 可靠性的 QoS 配置
        auto depth_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        depth_subscription_ =  this->create_subscription<std_msgs::msg::Float32>(
            "/YellowBot/depth", depth_qos, std::bind(&vertical_scan::depth_callback, this, std::placeholders::_1)); 
    }
    ~vertical_scan(){
        RCLCPP_INFO(this->get_logger(), "vertical_scan over.");
        std::string filename = folder_name+"/depths.csv";
        saveVectorToDepths(depth_vector,filename);
    }

private:

    void depth_callback(const std_msgs::msg::Float32::SharedPtr msg){
        current_depth = msg->data;
        RCLCPP_INFO(this->get_logger(), "current depth is %f", current_depth);
    }

        // 收到 m750d ping 的回调函数
    void m750d_ping_callback(const oculus_interfaces::msg::Ping::SharedPtr msg)
    {
        float depth_now = current_depth;
        if(depth_now >=1 && depth_now <= 7){
            RCLCPP_INFO(this->get_logger(), "save ping ");
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

            std::ostringstream index_str;
            index_str << std::setw(6) << std::setfill('0') << index;
            std::string index_string = index_str.str();
            std::string filename = folder_name+"/SonarRawData_" + index_string + ".csv";
            std::string filename2 = folder_name+"/Beams_"+ index_string + ".csv";
            index = index + 1;
            // 保存为CSV文件
            saveVectorToCSV(range_beam_map, filename);
            saveVectorToBeams(bearings,filename2);
            depth_vector.push_back(-depth_now);
        }
    }
};

int main(int argc, char* argv[])
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<vertical_scan>("vertical_scan");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "vertical_scan节点已经启动.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}