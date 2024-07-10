#ifndef __IMU_NODE_HPP__
#define __IMU_NODE_HPP__

#include "imu/100D4/100D4.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr auto DEFAULT_IMU_TOPIC = "imu";

class ImuNode : public rclcpp::Node
{

public:
    ImuNode()
    : Node("imu_node")
    {
        //声明参数
        this->declare_parameter<bool>("enable", enable_);
        this->declare_parameter<bool>("isMagnetic", isMagnetic_);
        this->declare_parameter<std::uint8_t>("frequency", frequency_);
        this->declare_parameter<std::string>("port", port_);
        this->declare_parameter<int>("baudrate", baudrate_);
        this->get_parameter("enable", enable_);
        this->get_parameter("isMagnetic", isMagnetic_);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);

        if(enable_)
        {
            while (rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "IMU seial connecting...");
                try
                {
                    imu_ser_.setPort(port_);
                    imu_ser_.setBaudrate(baudrate_);
                    auto timeout = serial::Timeout::simpleTimeout(1000/frequency_
                    );
                    imu_ser_.setTimeout(timeout);
                    imu_ser_.open();
                    if(!imu_ser_.isOpen())
                        RCLCPP_ERROR(this->get_logger(), "IMU seial opened failed!");
                    RCLCPP_INFO(this->get_logger(), "IMU seial opened sucessfully!");
                    break;
                }
                catch (serial::IOException& e)
                {
                    RCLCPP_WARN(this->get_logger(), "IMU seial connected failed! Try again in 1s...");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            //实例化
            imu_ptr_ = new SC100D4(&imu_ser_);
            //是否开启磁校准
            imu_ptr_->set_magnetic(isMagnetic_);
            //启动SC100D4
            imu_ptr_->set_status(true);
            
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(DEFAULT_IMU_TOPIC, rclcpp::SystemDefaultsQoS());

            timer_ms = std::chrono::milliseconds(1000/frequency_);
            
            timer_ = this->create_wall_timer(
                timer_ms, std::bind(&ImuNode::timer_callback, this));
        }
    }

private:
    void timer_callback()
    {
        int count = imu_ser_.available();
        uint8_t* read_buf = new uint8_t[count];
        while(count > 0)
        {
            uint8_t num = imu_ser_.read(read_buf, 1);
            if(num >0 && read_buf[0] == SC100D4::IMU_HEAD_SOF>>8)
            {
                //get remain data
                num = imu_ser_.read(read_buf + 1, SC100D4::IMU_DATA_LENGTH - 1);
                if (num == SC100D4::IMU_DATA_LENGTH - 1)
                {
                    imu_ptr_->imu_data_solve(read_buf);
                    sensor_msgs::msg::Imu imu_msg;
                    rclcpp::Time now = this->get_clock()->now();
                    imu_msg.header.stamp = now;
                    imu_msg.header.frame_id = "imu_link";

                    imu_msg.linear_acceleration.x = imu_ptr_->get_imu_data().Acc_x;
                    imu_msg.linear_acceleration.y = imu_ptr_->get_imu_data().Acc_y;
                    imu_msg.linear_acceleration.z = imu_ptr_->get_imu_data().Acc_z;
                    imu_msg.linear_acceleration_covariance = std::array<double, 9>{-1,0,0,0,0,0,0,0,0};

                    imu_msg.angular_velocity.x = imu_ptr_->get_imu_data().Gyrol_x;
                    imu_msg.angular_velocity.y = imu_ptr_->get_imu_data().Gyrol_y;
                    imu_msg.angular_velocity.z = imu_ptr_->get_imu_data().Gyrol_z;
                    imu_msg.angular_velocity_covariance = std::array<double, 9>{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

                    imu_msg.orientation.w = imu_ptr_->get_imu_data().quaternion[0];
                    imu_msg.orientation.x = imu_ptr_->get_imu_data().quaternion[1];
                    imu_msg.orientation.y = imu_ptr_->get_imu_data().quaternion[2];
                    imu_msg.orientation.z = imu_ptr_->get_imu_data().quaternion[3];
                    imu_msg.orientation_covariance = std::array<double, 9>{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

                    imu_publisher_->publish(imu_msg);
                }   
            }
            count--;
        }
        delete[] read_buf;
    }
    std::string port_;
    uint32_t baudrate_ = 115200;
     //频率
    uint8_t frequency_ = 100;
    bool enable_ = false;
    //磁修正
    bool isMagnetic_ = true;
    
    std::chrono::milliseconds timer_ms;
    rclcpp::TimerBase::SharedPtr timer_;

    serial::Serial imu_ser_;

    SC100D4* imu_ptr_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_ = nullptr;
};

#endif /* __IMU_NODE_HPP__ */
