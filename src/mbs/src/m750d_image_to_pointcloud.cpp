#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>


class m750d_img2pc : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m750d_image_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m750d_pointcloud_publisher_;

public:
    // 构造函数,有一个参数为节点名称
    m750d_img2pc(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());


        m750d_image_subscribe_ = this->create_subscription<sensor_msgs::msg::Image>("/blueview_p900/sonar_image", 10, \
                                    std::bind(&m750d_img2pc::m750d_image_callback, this, std::placeholders::_1));
        m750d_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("m750d_pointcloud", 10);

    }

    // 收到 m750d image 的回调函数
    void m750d_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "receive img");
    }



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<m750d_img2pc>("m750d_img2pc");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

