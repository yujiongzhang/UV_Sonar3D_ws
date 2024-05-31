#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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



class pc_stitch : public rclcpp::Node
{
private:

public:
    pc_stitch(std::string name) : Node(name)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zyj/Documents/GitHub/UV_Sonar3D_ws/results/3D1.pcd", *cloud1) == -1 ||
            pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zyj/Documents/GitHub/UV_Sonar3D_ws/results/3D2.pcd", *cloud2) == -1 ||
            pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zyj/Documents/GitHub/UV_Sonar3D_ws/results/3D3.pcd", *cloud3) == -1)
        {
            PCL_ERROR("Couldn't read input files\n");
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        // Define the transformation (translation and rotation)
        Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
        float x_translation = 7.5 ; // Example translation in x-direction
        float y_translation = 3.5 ; // Example translation in y-direction
        float z_translation = 0.0; // Example translation in z-direction
        float theta = - M_PI * 2 / 3; // Example rotation angle (120 degrees)
        transform2(0, 3) = x_translation;
        transform2(1, 3) = y_translation;
        transform2(2, 3) = z_translation;

        transform2.block<3, 3>(0, 0) = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()).toRotationMatrix();

        // Apply the transformation to the input cloud
        pcl::transformPointCloud(*cloud2, *transformed_cloud2, transform2);


        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
        // Define the transformation (translation and rotation)
        Eigen::Matrix4f transform3 = Eigen::Matrix4f::Identity();
        transform3(0, 3) = 7.5;
        transform3(1, 3) = - 3.5 ;
        transform3(2, 3) = 0.0;
        transform3.block<3, 3>(0, 0) = Eigen::AngleAxisf((M_PI * 2 / 3), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        // Apply the transformation to the input cloud
        pcl::transformPointCloud(*cloud3, *transformed_cloud3, transform3);


        *merged_cloud = *cloud1 + *transformed_cloud2 + *transformed_cloud3; // Merge the point clouds


        time_t timep;
        char file_name[256] = {0};
        time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
        strftime( file_name, sizeof(file_name), "./results/%Y.%m.%d %H-%M-%S.pcd",localtime(&timep) ); //最后一个参数是用localtime将秒数转化为struct tm结构体

        pcl::io::savePCDFile(std::string(file_name), *merged_cloud);

        pcl::visualization::PCLVisualizer viewer("display");
        //原始点云绿色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(merged_cloud, 100, 100, 255);
        viewer.setBackgroundColor(255, 255, 255);
        viewer.addPointCloud(merged_cloud, blue, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); //设置点云大小
        viewer.spin();
    }
    ~pc_stitch(){

    }

private:


};

int main(int argc, char* argv[])
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<pc_stitch>("pc_stitch");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "pc_stitch节点已经启动.");

    return 0;
}