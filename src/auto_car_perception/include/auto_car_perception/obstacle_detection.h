#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>

class ObstacleDetectionNode {
public:
    ObstacleDetectionNode();
    
private:
    // ROS 回调函数
    void carPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void semanticCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // 辅助函数
    void checkObstacleDistances(const geometry_msgs::PoseArray& obstacles);
    void updateTrackedObstacles(geometry_msgs::PoseArray& obstacles, const ros::Time& stamp);
    
    // ROS 成员变量
    ros::Subscriber camera_info_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber semantic_sub;
    ros::Subscriber car_pose_sub;
    ros::Publisher obstacles_pub;
    ros::Publisher obstacle_slow_pub;
    ros::Publisher obstacle_stop_pub;
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    
    // 图像数据
    sensor_msgs::CameraInfo camera_info;
    cv::Mat depth_image;
    bool camera_info_received;
    bool depth_received;
    bool car_pose_received;
    
    // 参数
    std::string camera_frame;
    std::string world_frame;
    int h_min;
    int h_max;
    int s_min;
    int v_min;
    float depth_min_valid;
    float depth_max_valid;
    double scale_x;
    double scale_y;
    bool resolution_mismatch;
    double stop_distance;          // 停车距离 (10米)
    double stop_angle_threshold;   // 停车角度阈值 (1度)
    double slow_distance;          // 减速距离
    double front_angle_threshold;  // 前方角度阈值

    bool obstacle_stop_state_;             // 当前停车状态
    ros::Time obstacle_stop_start_time_;   // 停车状态开始时间
    double obstacle_stop_duration_;        // 停车状态持续时间(秒)
    
    // 车辆位姿
    geometry_msgs::PoseStamped current_car_pose;
};

#endif // OBSTACLE_DETECTION_H