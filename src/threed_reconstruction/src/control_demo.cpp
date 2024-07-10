#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ControlDemo : public rclcpp::Node
{
public:
    ControlDemo() : Node("control_demo_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),//10Hz
            std::bind(&ControlDemo::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("world", "sparus",
                                                           tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "Transform from map to base_footprint: \nPosition: [%.2f, %.2f, %.2f]",
                        transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform world to sparus: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlDemo>());
    rclcpp::shutdown();
    return 0;
}
