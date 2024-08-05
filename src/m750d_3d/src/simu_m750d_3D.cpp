#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "cv_bridge/cv_bridge.h"
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


class m750d_3D : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    double ScanRange;
    double PixelResolution;
    volatile double x_p;
    volatile double y_p;
    volatile double z_p;
    volatile double yaw_p;
    Eigen::Matrix4f transformation;

public:
    m750d_3D(std::string name) : Node(name)
    {
        pointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        ScanRange = 10.0;
        z_p = 0;
        transformation = Eigen::Matrix4f::Identity();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oculus_m1200d/sonar_image", 10, std::bind(&m750d_3D::image_callback, this, std::placeholders::_1));
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/oculus_m750d/pose",10,std::bind(&m750d_3D::pose_callback, this, std::placeholders::_1));
    }
    ~m750d_3D(){
        RCLCPP_INFO(this->get_logger(), "m750d_3D over.");

        time_t timep;
        char name[256] = {0};
        time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
        strftime( name, sizeof(name), "./results/pier/%Y.%m.%d %H-%M-%S.pcd",localtime(&timep) ); //最后一个参数是用localtime将秒数转化为struct tm结构体

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
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "pose is ");
        x_p = msg->position.x;
        y_p = msg->position.y;
        z_p = msg->position.z;
        double ox = msg->orientation.x;
        double oy = msg->orientation.y;
        double oz = msg->orientation.z;
        double ow = msg->orientation.w;
        yaw_p = atan2(2.0f * (ow * oz + ox * oy), 1.0f - 2.0f * (oy * oy + oz * oz));
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将ROS 2图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 在这里对OpenCV图像进行任何所需的处理
            cv::Mat raw_image = cv_ptr->image;
            RCLCPP_INFO(this->get_logger(), "image size rows :%d , cols: %d" , raw_image.rows, raw_image.cols);
            
            
            // 将彩色图像转换为灰度图像
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

            // //图像增强
            // cv::Mat enhance_image(gray_image.size(),gray_image.type());
            // // powerTrans(gray_image, enhance_image, 0.2, 2);
            // cv::equalizeHist(gray_image, enhance_image);// 应用直方图均衡化

            // //图像降噪
            // cv::Mat bila_res;
            // cv::bilateralFilter(enhance_image, bila_res, 24, 24 * 2, 24 / 2);

            // //形态学处理
            // cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
            // cv::Mat close_image;
            // morphologyEx(bila_res, close_image, cv::MORPH_CLOSE, element);

            //提取有效轮廓
            Mat edge_effe(gray_image.size(), gray_image.type(), Scalar(0));//clear 0 
            edgeExtraction(gray_image, edge_effe, 100);
            
            int width = edge_effe.cols;
            int height = edge_effe.rows;

            // 显示图像
            cv::imshow("raw Image", raw_image);
            cv::imshow("Gray Image", gray_image);
            // cv::imshow("enhance Image", enhance_image);
            // cv::imshow("bila_res", bila_res);
            // cv::imshow("close_image", close_image);
            cv::imshow("edge_effe", edge_effe);

            cv::waitKey(1); // 等待按键输入

            int origin_x = width/2;
            int origin_y = height;
            PixelResolution = ScanRange / (double)height;
            for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        cv::Point2d point_index(x,y);
                        // if (edge_effe.at<uchar>(point_index) > 0)
                        if ( z_p<=-5 && z_p >= -29 && edge_effe.at<uchar>(point_index) > 100)
                        {
                            int x_p_ = origin_y - y;
                            int y_p_ = origin_x - x;

                            pcl::PointXYZ p;
                            pcl::PointXYZ p_trans;

                            p.x = x_p_ * PixelResolution;
                            p.y = y_p_ * PixelResolution; 
                            p.z = z_p;

                            // 将点转换为Eigen的Vector4f，以便使用变换矩阵
                            Eigen::Vector4f pointVec(p.x, p.y, p.z, 1.0f);
                            // //
                            // Eigen::Vector3f translation(x_p, y_p, 0);
                            // // 定义旋转（绕Z轴旋转30度）
                            // Eigen::AngleAxisf rotation( yaw_p , Eigen::Vector3f::UnitZ());
                            // Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();
                            // transformation.block<3,1>(0,3) = translation;
                            // transformation.block<3,3>(0,0) = rotationMatrix;

                            Eigen::Matrix4f transform3 = Eigen::Matrix4f::Identity();
                            transform3(0, 3) = x_p;
                            transform3(1, 3) = y_p;
                            transform3(2, 3) = 0.0;
                            transform3.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw_p, Eigen::Vector3f::UnitZ()).toRotationMatrix();

                            // 应用变换
                            Eigen::Vector4f transformedPointVec = transform3 * pointVec;
                            p_trans.x = transformedPointVec.x();
                            p_trans.y = transformedPointVec.y();
                            p_trans.z = transformedPointVec.z();

                            pointCloud->push_back(p_trans);
                        }
                    }
                }

        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }


};

int main(int argc, char* argv[])
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<m750d_3D>("m750d_3D");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "m750d_3D节点已经启动.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}