#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <auto_car_perception/ObstacleState.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <simulation/VehicleControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <limits> 


struct Area {
    double x_min, x_max;
    double y_min, y_max;

    bool contains(double x, double y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

class ColorDetectionNode
{
public:
    ColorDetectionNode();

private:
    void rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void semanticCameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void detectColorsInROI(const cv::Mat& roi);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // ROS通信对象
    ros::Subscriber rgb_camera_sub;
    ros::Subscriber semantic_camera_sub;
    ros::Subscriber depth_camera_sub;
    ros::Subscriber pose_sub;
    ros::Publisher result_pub;
    ros::Publisher stop_pub;
    ros::Publisher vehicle_control_pub;
    ros::Publisher traffic_light_presence_pub;

    // RGB图
    cv::Mat rgb_image;
    bool rgb_received;

    // 深度图
    cv::Mat depth_image;
    bool depth_received;

    // 检测区域
    bool in_detection_area = false;
    std::vector<Area> detection_areas;
    
};

#endif
