#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"



class adis_imu_process : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr adis_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr adis_publisher_;
    // sensor_msgs::msg::Imu adis_msg;

public:
    // 构造函数,有一个参数为节点名称
    adis_imu_process(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        adis_subscribe_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu_adis_ros", 10, \
                                    std::bind(&adis_imu_process::adis_callback, this, std::placeholders::_1));
        adis_publisher_ =  this->create_publisher<sensor_msgs::msg::Imu>("adis_imu_covariance", 10);
    }

    // 收到 m750d ping 的回调函数
    void adis_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "imu_adis_ros has received");
        sensor_msgs::msg::Imu temp = *msg;
        temp.orientation_covariance.fill(0.0);
        temp.orientation_covariance[0*3+0] = 1e-6;
        temp.orientation_covariance[1*3+1] = 1e-6;
        temp.orientation_covariance[2*3+2] = 1e-6;
        temp.angular_velocity_covariance.fill(0.0);
        temp.angular_velocity_covariance[0*3+0] = 1e-6;
        temp.angular_velocity_covariance[1*3+1] = 1e-6;
        temp.angular_velocity_covariance[2*3+2] = 1e-6;
        temp.linear_acceleration_covariance.fill(0.0);
        temp.linear_acceleration_covariance[0*3+0] = 1e-6;
        temp.linear_acceleration_covariance[1*3+1] = 1e-6;
        temp.linear_acceleration_covariance[2*3+2] = 1e-6;

        adis_publisher_->publish(temp);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<adis_imu_process>("adis_imu_process");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

