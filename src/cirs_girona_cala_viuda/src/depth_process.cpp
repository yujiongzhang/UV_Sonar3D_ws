#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cirs_girona_cala_viuda/msg/depth.hpp"



class depth_process : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<cirs_girona_cala_viuda::msg::Depth>::SharedPtr depth_subscribe_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_publisher_;
    geometry_msgs::msg::PoseWithCovarianceStamped depth_msg;

public:
    // 构造函数,有一个参数为节点名称
    depth_process(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        depth_subscribe_ = this->create_subscription<cirs_girona_cala_viuda::msg::Depth>("/depth_sensor", 10, \
                                    std::bind(&depth_process::Depth_callback, this, std::placeholders::_1));
        depth_publisher_ =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_z", 10);

        depth_msg.pose.covariance.fill(0.0); // 对应6x6的协方差矩阵
        depth_msg.pose.covariance[6*0 + 0] =  1e6;
        depth_msg.pose.covariance[6*1 + 1] =  1e6;
        depth_msg.pose.covariance[6*2 + 2] =  1e-4;// 0.01 * 0.01;
        depth_msg.pose.covariance[6*3 + 3] =  1e6;
        depth_msg.pose.covariance[6*4 + 4] =  1e6;
        depth_msg.pose.covariance[6*5 + 5] =  1e6;
    }

    // 收到 m750d ping 的回调函数
    void Depth_callback(const cirs_girona_cala_viuda::msg::Depth::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Depth has received");
        depth_msg.header = msg->header;
        depth_msg.header.frame_id = "world";
        depth_msg.pose.pose.position.z = msg->depth;
        depth_publisher_->publish(depth_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<depth_process>("depth_process");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

