#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include <vector>
using namespace std;
using namespace cv;
//class mycomparison;

void dynamicBrightness(vector<vector<uchar>>& dataMatrix_step2,double index);
void calDeltaXY();
void ringScanForEdge(const cv::Mat& image, cv::Mat& edge, int threshold);
void ringScanForEdge_Sector(const cv::Mat& image, cv::Mat& edge, int threshold);
void linearTrans(cv::Mat& src, cv::Mat& dst, double a, double b);
void powerTrans(cv::Mat& src, cv::Mat& dst, double para, double exp);

void edgeExtraction(const cv::Mat& image, cv::Mat& edge, int threshold);
