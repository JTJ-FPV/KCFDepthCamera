#include <ros/ros.h>
#include <Camera/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>


// static const char WINDOW[] = "IMGAE VIEWER";
// KCF初始化参数
const bool HOG(true), FIXWINDOW(true), MULTISCALE(true), SILENT(true), LAB(true), DSST = false;
const int MINIMUM_POINTS = 1, EPSILON = 50;
const int MINIMUM_POINTS_1D = 10, EPSILON_1D = 0.015 * 0.015;
const int MINIMUM_POINTS_2D = 10, EPSILON_2D = 0.9 * 0.9;//0.0007 * 0.0007;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "CADC_Camera");
    ros::NodeHandle nh;
   
    // *************simulation*************
    // AAMED aam_l(1080, 1920);
    // AAMED aam_r(1080, 1920);
    // *************simulation*************

    // *************realflight*************
    AAMED aam(720, 1280);
    // *************realflight*************
    kcf::KCFTracker tra(HOG, FIXWINDOW, MULTISCALE, LAB, DSST);
    DBSCAN::DBSCAN d(MINIMUM_POINTS, EPSILON);
    DBSCAN::DBSCAN d_1(MINIMUM_POINTS_1D, EPSILON_1D);
    DBSCAN::DBSCAN d_2(MINIMUM_POINTS_2D, EPSILON_2D);
    cv::namedWindow("IMGAE COLOR VIEWER");
    cv::namedWindow("IMGAE DEPTH VIEWER");
    cv::startWindowThread();
    ROS_INFO("main before");
    // CADC::camera cam(&nh, aam, tra, d, 0, 0, 0.7);
    CADC::DepthCamera cam(&nh, aam, tra, d, d_1, d_2, 10, 0, 0, 0);
    ROS_INFO("main after");
    ros::spin();
    return 0;
}
