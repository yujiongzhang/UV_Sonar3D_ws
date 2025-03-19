#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
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



class vertical_scan : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m750d_edge_pointcloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    double ScanRange;
    double PixelResolution;
    double z_p;
    volatile float current_depth;
    float start_yaw;
    volatile float current_yaw;

public:
    vertical_scan(std::string name) : Node(name)
    {
        pointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        ScanRange = 10.0;
        z_p = 0;
        start_yaw = 0;
        current_yaw = 0;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oculus_m1200d/sonar_image", 10, std::bind(&vertical_scan::image_callback, this, std::placeholders::_1));

        m750d_edge_pointcloud_subscription_ =  this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sonar/edge_points", 10, std::bind(&vertical_scan::points_callback, this, std::placeholders::_1)); 

        // 创建一个具有 Best Effort 可靠性的 QoS 配置
        auto depth_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        depth_subscription_ =  this->create_subscription<std_msgs::msg::Float32>(
            "/YellowBot/depth", depth_qos, std::bind(&vertical_scan::depth_callback, this, std::placeholders::_1)); 
        
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/YellowBot/imu", 10, std::bind(&vertical_scan::imu_callback, this, std::placeholders::_1));
    }
    ~vertical_scan(){
        RCLCPP_INFO(this->get_logger(), "vertical_scan over.");

        time_t timep;
        char name[256] = {0};
        time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
        strftime( name, sizeof(name), "./results/%Y.%m.%d %H-%M-%S.pcd",localtime(&timep) ); //最后一个参数是用localtime将秒数转化为struct tm结构体

        // pcl::io::savePCDFile("./sonar3D.pcd", *pointCloud);
        pcl::io::savePCDFile(std::string(name), *pointCloud);

        pcl::visualization::PCLVisualizer viewer("display");
        //原始点云绿色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(pointCloud, 100, 100, 255);
        viewer.setBackgroundColor(255, 255, 255);
        viewer.addPointCloud(pointCloud, blue, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); //设置点云大小
        viewer.spin();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // try
        // {
        //     // 将ROS 2图像消息转换为OpenCV图像
        //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        //     // 在这里对OpenCV图像进行任何所需的处理
        //     cv::Mat raw_image = cv_ptr->image;
        //     RCLCPP_INFO(this->get_logger(), "image size rows :%d , cols: %d" , raw_image.rows, raw_image.cols);
            
            
        //     // 将彩色图像转换为灰度图像
        //     cv::Mat gray_image;
        //     cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        //     //图像增强
        //     cv::Mat enhance_image(gray_image.size(),gray_image.type());
        //     // powerTrans(gray_image, enhance_image, 0.2, 2);
        //     cv::equalizeHist(gray_image, enhance_image);// 应用直方图均衡化

        //     //图像降噪
        //     cv::Mat bila_res;
        //     cv::bilateralFilter(enhance_image, bila_res, 24, 24 * 2, 24 / 2);

        //     //形态学处理
        //     cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
        //     cv::Mat close_image;
        //     morphologyEx(bila_res, close_image, cv::MORPH_CLOSE, element);

        //     //环扫提取有效轮廓
        //     Mat edge_effe(close_image.size(), close_image.type(), Scalar(0));//clear 0 
        //     edgeExtraction(close_image, edge_effe, 100);
            
        //     int width = edge_effe.cols;
        //     int height = edge_effe.rows;

        //     // 显示图像
        //     cv::imshow("raw Image", raw_image);
        //     cv::imshow("Gray Image", gray_image);
        //     cv::imshow("enhance Image", enhance_image);
        //     cv::imshow("bila_res", bila_res);
        //     cv::imshow("close_image", close_image);
        //     cv::imshow("edge_effe", edge_effe);

        //     cv::waitKey(1); // 等待按键输入


        //     int origin_x = width/2;
        //     int origin_y = height;
        //     z_p += 0.02; 
        //     PixelResolution = ScanRange / (double)height;
        //     for (int x = 0; x < width; x++)
        //         {
        //             for (int y = 0; y < height; y++)
        //             {
        //                 cv::Point2d point_index(x,y);
        //                 // if (edge_effe.at<uchar>(point_index) > 0)
        //                 if (gray_image.at<uchar>(point_index) > 100)
        //                 {
        //                     int x_p = origin_y - y;
        //                     int y_p = origin_x - x;

        //                     pcl::PointXYZ p;
        //                     p.x = x_p * PixelResolution;
        //                     p.y = y_p * PixelResolution; 
        //                     p.z = z_p; 
        //                     pointCloud->push_back(p);
        //                 }
        //             }
        //         }

        // }
        // catch (cv_bridge::Exception& e)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        // }
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        // 创建一个PCL类型的点云
        if(current_depth >= 1 && current_depth <=7 ){
            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *new_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_depth(new pcl::PointCloud<pcl::PointXYZ>);
            // Define the transformation (translation and rotation)
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            float x_translation = 0.0 ; // Example translation in x-direction
            float y_translation = 0.0 ; // Example translation in y-direction
            float z_translation = - current_depth; // Example translation in z-direction
            // float theta =  current_yaw - start_yaw; // Example rotation angle (120 degrees)
            // RCLCPP_INFO(this->get_logger(), "theta is %f", theta);
            transform(0, 3) = x_translation;
            transform(1, 3) = y_translation;
            transform(2, 3) = z_translation;
            // transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()).toRotationMatrix();
            // Apply the transformation to the input cloud
            pcl::transformPointCloud(*new_cloud, *new_cloud_depth, transform);

            *pointCloud = *pointCloud + *new_cloud_depth;
        }

    }

    void depth_callback(const std_msgs::msg::Float32::SharedPtr msg){
        current_depth = msg->data;
        RCLCPP_INFO(this->get_logger(), "current depth is %f", current_depth);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        double q_x = msg->orientation.x;
        double q_y = msg->orientation.y;
        double q_z = msg->orientation.z;
        double q_w = msg->orientation.w; 
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z);
        current_yaw = std::atan2(siny_cosp, cosy_cosp);

        if( start_yaw==0 && current_depth >= 1 && current_depth <=7 ){
            start_yaw = current_yaw;
        }
        // RCLCPP_INFO(this->get_logger(), "current_yaw is %f ,start_yaw is %f", current_yaw,start_yaw);
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