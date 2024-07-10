#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl_conversions/pcl_conversions.h>


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



class cave_recon : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;

    std::array<double, 3>  cur_pose;
    std::array<double, 4>  cur_orientation;


public:
    cave_recon(std::string name) : Node(name)
    {
        pointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cur_pose = {0.0, 0.0, 0.0};
        cur_orientation = {0.0, 0.0, 0.0, 1.0};

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/sonar_seaking_ros", 10, std::bind(&cave_recon::seaking_callback, this, std::placeholders::_1));

        odometry_subscription_ =  this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&cave_recon::odometry_callback, this, std::placeholders::_1)); 

    }
    ~cave_recon(){
        RCLCPP_INFO(this->get_logger(), "cave_recon over.");

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
    void seaking_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {

    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        cur_pose[0] = msg->pose.pose.position.x;
        cur_pose[1] = msg->pose.pose.position.y;
        cur_pose[2] = msg->pose.pose.position.z;

        cur_orientation[0] = msg->pose.pose.orientation.x;
        cur_orientation[1] = msg->pose.pose.orientation.y;
        cur_orientation[2] = msg->pose.pose.orientation.z;
        cur_orientation[3] = msg->pose.pose.orientation.w;
        RCLCPP_INFO(this->get_logger(), "current position is %f, %f, %f",cur_pose[0],cur_pose[1],cur_pose[2] );
    }
};

int main(int argc, char* argv[])
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<cave_recon>("cave_recon");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "cave_recon节点已经启动.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}