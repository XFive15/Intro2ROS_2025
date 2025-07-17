#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ObstacleDetectionNode {
public:
    ObstacleDetectionNode();

private:
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void semanticCallback(const sensor_msgs::ImageConstPtr& msg);
    void carPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void showDebugWindows(const cv::Mat& semantic_img, const cv::Mat& mask);
    void publishObstacleMarkers(const geometry_msgs::PoseArray& obstacles);
    void checkObstacleDistances(const geometry_msgs::PoseArray& obstacles);
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    
    ros::Subscriber camera_info_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber semantic_sub;
    ros::Subscriber car_pose_sub;
    ros::Publisher obstacles_pub;
    ros::Publisher markers_pub;
    ros::Publisher obstacle_slow_pub;
    ros::Publisher obstacle_stop_pub;
    
    sensor_msgs::CameraInfo camera_info;
    cv::Mat depth_image;
    geometry_msgs::PoseStamped current_car_pose;
    
    bool camera_info_received;
    bool depth_received;
    bool car_pose_received;
    
    std::string camera_frame;
    std::string world_frame;
    
    int h_min, h_max, s_min, v_min;
    float depth_min_valid, depth_max_valid;
    double scale_x, scale_y;
    bool resolution_mismatch;
    
    // Distance parameters
    double stop_distance;
    double slow_distance;
    double front_angle_threshold;
};

#endif // OBSTACLE_DETECTION_H