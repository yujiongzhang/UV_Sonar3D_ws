#ifndef __DEPTH_NODE_HPP__
#define __DEPTH_NODE_HPP__

#include "depth/keller/keller.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/temperature.hpp"

constexpr auto DEFAULT_DEPTH_TOPIC = "depth";

class DepthNode : public rclcpp::Node
{

public:
    DepthNode()
    : Node("depth_node")
    {
        //声明参数
        this->declare_parameter<bool>("enable", enable_);
        this->declare_parameter<std::uint8_t>("frequency", frequency_);
        this->declare_parameter<std::string>("port", port_);
        this->declare_parameter<int>("baudrate", baudrate_);
        this->declare_parameter<double>("depth_covariance", depth_covariance_);
        this->get_parameter("enable", enable_);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("depth_covariance", depth_covariance_);

        if(enable_)
        {
            while (rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Depth seial connecting...");
                try
                {
                    depth_ser_.setPort(port_);
                    depth_ser_.setBaudrate(baudrate_);
                    auto timeout = serial::Timeout::simpleTimeout(1000/frequency_);
                    depth_ser_.setTimeout(timeout);
                    depth_ser_.open();
                    if(!depth_ser_.isOpen())
                        RCLCPP_ERROR(this->get_logger(), "Depth seial opened failed!");
                    RCLCPP_INFO(this->get_logger(), "Depth seial opened sucessfully!");
                    break;
                }
                catch (serial::IOException& e)
                {
                    RCLCPP_WARN(this->get_logger(), "Depth seial connected failed! Try again in 1s...");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            //实例化
            depth_ptr_ = new KELLER(&depth_ser_, 0x01);
            // Create a publisher
            depth_publisher_ = this->create_publisher<std_msgs::msg::Float32>(DEFAULT_DEPTH_TOPIC, rclcpp::SystemDefaultsQoS());
            externTemp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", rclcpp::SystemDefaultsQoS());
            pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_z", rclcpp::SystemDefaultsQoS());

            timer_ms = std::chrono::milliseconds(1000/frequency_);
            
            timer_ = this->create_wall_timer(
                timer_ms, std::bind(&DepthNode::timer_callback, this));
        }
    }

private:
    void timer_callback()
    {
        //请求一次数据
        depth_ptr_->request_data();
        //阻塞直至接收到数据或者超时
        if(depth_ser_.waitReadable())
        {
            int count = depth_ser_.available();
            uint8_t* read_buf = new uint8_t[count];
            while(count > 0)
            {
                uint8_t num = depth_ser_.read(read_buf, 1);
                if(num >0 && read_buf[0] == KELLER::DEPTH_HEAD_SOF)
                {
                    //get remain data
                    num = depth_ser_.read(read_buf + 1, KELLER::DEPTH_DATA_LENGTH - 1);
                    if (num == KELLER::DEPTH_DATA_LENGTH - 1)
                    {
                        depth_ptr_->depth_data_solve(read_buf);
                        std_msgs::msg::Float32 depth_msg;
                        depth_msg.data = depth_ptr_->get_depth_data();
                        depth_publisher_->publish(depth_msg);

                        rclcpp::Time now = this->get_clock()->now();
                        //发布舱外温度
                        sensor_msgs::msg::Temperature temp_msg;
                        temp_msg.header.stamp = now;
                        temp_msg.header.frame_id = "base_link";
                        temp_msg.temperature = depth_ptr_->get_temperature_data();
                        externTemp_publisher_->publish(temp_msg);
                        //发布坐标
                        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
                        pose_msg.header.stamp = now;
                        pose_msg.header.frame_id = "base_link";
                        pose_msg.pose.pose.position.z = -depth_ptr_->get_depth_data();
                        pose_msg.pose.covariance[14] = depth_covariance_;
                        pose_publisher_->publish(pose_msg);
                    }   
                }
                count--;
            }
            delete[] read_buf;
        }
        
    }
    std::string port_;
    uint32_t baudrate_ = 115200;
     //频率
    uint8_t frequency_ = 50;
    bool enable_ = false;
    
    std::chrono::milliseconds timer_ms;
    rclcpp::TimerBase::SharedPtr timer_;

    serial::Serial depth_ser_;

    KELLER* depth_ptr_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> depth_publisher_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature>> externTemp_publisher_ = nullptr;

    double depth_covariance_;
    //发布位置数据
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_publisher_ = nullptr;

};
#endif /* __DEPTH_NODE_HPP__ */

