#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "cirs_girona_cala_viuda/msg/linkquest_dvl.hpp"



class dvl_process : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<cirs_girona_cala_viuda::msg::LinkquestDvl>::SharedPtr LinkquestDvl_subscribe_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_publisher_;
    geometry_msgs::msg::TwistWithCovarianceStamped dvl_msg;

public:
    // 构造函数,有一个参数为节点名称
    dvl_process(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        LinkquestDvl_subscribe_ = this->create_subscription<cirs_girona_cala_viuda::msg::LinkquestDvl>("/dvl_linkquest", 10, \
                                    std::bind(&dvl_process::LinkquestDvl_callback, this, std::placeholders::_1));
        dvl_publisher_ =  this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl2", 10);

        dvl_msg.twist.covariance.fill(0.0); // 对应6x6的协方差矩阵
        dvl_msg.twist.covariance[6*0 + 0] =  1e-6;
        dvl_msg.twist.covariance[6*1 + 1] =  1e-6;
        dvl_msg.twist.covariance[6*2 + 2] =  1e-6;// 0.1 * 0.1;
        dvl_msg.twist.covariance[6*3 + 3] =  1e6;
        dvl_msg.twist.covariance[6*4 + 4] =  1e6;
        dvl_msg.twist.covariance[6*5 + 5] =  1e6;
    }

    // 收到 m750d ping 的回调函数
    void LinkquestDvl_callback(const cirs_girona_cala_viuda::msg::LinkquestDvl::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "linkquestdvl has received");

        if(msg->velocity_earth_flag == 1){
            dvl_msg.header = msg->header;
            // dvl_msg.header.frame_id = "world";
            dvl_msg.twist.twist.linear.x = msg->velocity_earth[0];
            dvl_msg.twist.twist.linear.y = msg->velocity_earth[1];
            dvl_msg.twist.twist.linear.z = msg->velocity_earth[2];
            dvl_publisher_->publish(dvl_msg);
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<dvl_process>("dvl_process");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

