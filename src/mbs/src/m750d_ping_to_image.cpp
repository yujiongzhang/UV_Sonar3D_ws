#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "oculus_interfaces/msg/ping.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>




class m750d_ping2img : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr m750d_ping_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m750d_image_publisher_;

public:
    // 构造函数,有一个参数为节点名称
    m750d_ping2img(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        m750d_ping_subscribe_ = this->create_subscription<oculus_interfaces::msg::Ping>("/sonar/ping", 10, \
                                    std::bind(&m750d_ping2img::m750d_ping_callback, this, std::placeholders::_1));
        m750d_image_publisher_ =  this->create_publisher<sensor_msgs::msg::Image>("m750d_image", 10);

    }

    // 收到 m750d ping 的回调函数
    void m750d_ping_callback(const oculus_interfaces::msg::Ping::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "receive /sonar/ping");
        const int width = msg->n_beams;
        const int height = msg -> n_ranges;
        const int step = msg->step;
        const std::vector<uint8_t> ping_data = msg->ping_data;
        const int offset = 2048;
        // const float theta_shift = 270; // θ偏移
        const int mat_encoding = CV_8U;
        const char* ros_image_encoding = sensor_msgs::image_encodings::MONO8;

        const double bearing = 65.0 * M_PI / 180;
        const float bearing_ratio = 2 * bearing / width;
        const int negative_height = static_cast<int>(std::floor(height * std::sin(-bearing)));
        const int positive_height = static_cast<int>(std::ceil(height * std::sin(bearing)));
        const int image_width = positive_height - negative_height;
        const int origin_width = abs(negative_height);  // x coordinate of the origin

        const cv::Size image_size(image_width, height);
        cv::Mat map(image_size, CV_32FC2);
        cv::parallel_for_(cv::Range(0, map.total()), [&](const cv::Range& range) {
            for (auto i = range.start; i < range.end; i++) {
                int y = i / map.cols;
                int x = i % map.cols;//将一维像素索引i转换为二维像素坐标(x, y)

                // Calculate range and bearing of this pixel from origin
                const float dx = x - origin_width;
                const float dy = map.rows - y;

                const float range = sqrt(dx * dx + dy * dy);
                const float bearing_x_y = atan2(dx, dy);
                // atan2(a,b)是4象限反正切，它的取值不仅取决于a/b的atan值，还取决于点 (b, a) 落入哪个象限
                //atan2 是求取反正切值的函数，它可以返回由给定的两个参数（dx 和 dy）得到的方位角，即与 x 轴正方向的夹角。

                float xp = range;
                // Linear interpolation, TODO: use a better interpolation method
                // 线性插值，TODO:使用更好的插值方法
                // float yp = (bearing_x_y + bearing) / bearing_ratio; // bearing_x_y : -bearing ~ bearing
                float yp = (bearing_x_y + bearing )/(2*bearing) * width;

                map.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
            }
        });

        cv::Mat source_map_1, source_map_2;
        cv::convertMaps(map, cv::Mat(), source_map_1, source_map_2, CV_16SC2);
/* 
这一行代码使用 OpenCV 的 convertMaps 函数将 map 转换成两个 CV_16SC2 格式的图像变换映射矩阵 source_map_1 和 source_map_2。
这一步是为了方便后续对图像进行几何变换，如透视变换、仿射变换等。
CV_16SC2：表示每个像素点由两个 16 位有符号整数组成。在这里，source_map_1 存储 x 方向的变换信息，source_map_2 存储 y 方向的变换信息。
将 map 转换成这种格式后，可以通过传递这两个变换矩阵给 OpenCV 的函数进行图像几何变换操作，从而实现对图像的变换和处理。
*/
        cv::Mat sonar_mat_data(height, step, mat_encoding);  // Note that the width is 'step' to include gain data
        // Copy the data including gain data
        for (int i = 0; i < height; ++i)
            std::copy(ping_data.begin() + offset + i * step, ping_data.begin() + offset + (i + 1) * step, sonar_mat_data.ptr<uint8_t>(i));

        // Now remove the gain data from sonar_mat_data
        cv::Mat sonar_mat_data_without_gain(height, width, mat_encoding);
        for (int i = 0; i < height; ++i)
            std::copy(sonar_mat_data.ptr<uint8_t>(i) + 4, sonar_mat_data.ptr<uint8_t>(i) + step,
                sonar_mat_data_without_gain.ptr<uint8_t>(i));


        RCLCPP_INFO_STREAM(this->get_logger(), "start picture");

        // 创建一个大小为 image_width x height 的矩阵，其中每个元素的值初始化为 1,将矩阵中的每个元素都乘以 uint8_t::max()，即最大值，这样可以将矩阵中所有元素的值设置为 uint8_t 类型的最大值。
        cv::Mat out = cv::Mat::ones(cv::Size(image_width, height), CV_MAKETYPE(mat_encoding, 1)) * std::numeric_limits<uint8_t>::max();

        cv::remap(sonar_mat_data_without_gain.t(), out, source_map_1, source_map_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT,
            cv::Scalar(std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()));
// cv::remap：OpenCV 中的重映射函数，用于根据给定的映射关系转换图像。
// sonar_mat_data_without_gain.t()：原始数据矩阵的转置，通常是为了满足 remap 函数的输入要求。
// out：存储重映射后的结果的输出矩阵。
// source_map_1 和 source_map_2：用于定义重映射的映射关系的两个输入矩阵，通常是通过对原始图像进行处理得到的。
// cv::INTER_CUBIC：指定重映射时使用的插值方法。最近邻插值 (cv::INTER_NEAREST).双线性插值 (cv::INTER_LINEAR),三次插值 (cv::INTER_CUBIC)
// cv::BORDER_CONSTANT：指定重映射时边界处理的方法，这里是使用常数边界。
// cv::Scalar(std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max())：用于常数边界处理的边界值，这里设置为 uint8_t 类型的最大值。

        // Publish sonar conic image
        sensor_msgs::msg::Image image_msg;
        cv_bridge::CvImage(msg->header, ros_image_encoding, out).toImageMsg(image_msg);
        m750d_image_publisher_->publish(image_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<m750d_ping2img>("m750d_ping2img");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

