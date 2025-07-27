#include "obstacle_detection.h"
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>
#include <auto_car_perception/ObstacleState.h>

ObstacleDetectionNode::ObstacleDetectionNode()
    : tf_listener(tf_buffer),
      camera_info_received(false),
      depth_received(false),
      car_pose_received(false),
      camera_frame("OurCar/Sensors/DepthCamera"),
      world_frame("world"),
      h_min(0),
      h_max(10),
      s_min(40),
      v_min(40),
      depth_min_valid(0.1f),
      depth_max_valid(100.0f),
      scale_x(2.0),
      scale_y(2.0),
      resolution_mismatch(false),
      stop_distance(15),
      stop_angle_threshold(15.0 * M_PI / 180.0),
      slow_distance(40.0),
      front_angle_threshold(20.0 * M_PI / 180.0),
      obstacle_stop_state_(false),
      obstacle_stop_start_time_(ros::Time(0)),
      obstacle_stop_duration_(5.0)
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("h_min", h_min, h_min);
    nh_private.param("h_max", h_max, h_max);
    nh_private.param("s_min", s_min, s_min);
    nh_private.param("v_min", v_min, v_min);
    nh_private.param("camera_frame", camera_frame, camera_frame);
    nh_private.param("world_frame", world_frame, world_frame);
    nh_private.param("depth_min_valid", depth_min_valid, depth_min_valid);
    nh_private.param("depth_max_valid", depth_max_valid, depth_max_valid);
    nh_private.param("stop_distance", stop_distance, stop_distance);
    
    double stop_angle_threshold_deg;
    nh_private.param("stop_angle_threshold_deg", stop_angle_threshold_deg, 1.0);
    stop_angle_threshold = stop_angle_threshold_deg * M_PI / 180.0;
    
    nh_private.param("slow_distance", slow_distance, slow_distance);
    nh_private.param("front_angle_threshold", front_angle_threshold, front_angle_threshold);
    
    nh_private.param("obstacle_stop_duration", obstacle_stop_duration_, obstacle_stop_duration_);

    camera_info_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info", 1,
                                   &ObstacleDetectionNode::cameraInfoCallback, this);
    depth_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1,
                             &ObstacleDetectionNode::depthCallback, this);
    semantic_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1,
                                &ObstacleDetectionNode::semanticCallback, this);
    car_pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1,
                                &ObstacleDetectionNode::carPoseCallback, this);

    obstacles_pub = nh.advertise<geometry_msgs::PoseArray>("/obstacles", 10);
    obstacle_slow_pub = nh.advertise<auto_car_perception::ObstacleState>("/obstacle_slow", 10);
    obstacle_stop_pub = nh.advertise<auto_car_perception::ObstacleState>("/obstacle_stop", 10);

    try {
        tf_buffer.canTransform(world_frame, camera_frame, ros::Time(0), ros::Duration(10.0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("TF check failed: %s", ex.what());
    }
}

void ObstacleDetectionNode::carPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_car_pose = *msg;
    car_pose_received = true;
}

void ObstacleDetectionNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info = *msg;
    camera_info_received = true;
}

void ObstacleDetectionNode::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image = cv_ptr->image;
        } 
        else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_ptr->image.convertTo(depth_image, CV_32FC1, 0.001);
        }
        else {
            ROS_ERROR("Unsupported depth encoding: %s", msg->encoding.c_str());
            return;
        }

        depth_received = true;

        cv::Mat mask_invalid = (depth_image > depth_max_valid) | (depth_image < depth_min_valid);
        depth_image.setTo(std::numeric_limits<float>::quiet_NaN(), mask_invalid);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error (depth): %s", e.what());
    }
}

void ObstacleDetectionNode::checkObstacleDistances(const geometry_msgs::PoseArray& obstacles) {
    bool current_stop = false;
    bool obstacle_slow = false;
    ros::Time now = ros::Time::now();

    double car_x = current_car_pose.pose.position.x;
    double car_y = current_car_pose.pose.position.y;
    double car_yaw = tf2::getYaw(current_car_pose.pose.orientation);

    for (const auto& obstacle : obstacles.poses) {
        double dx = obstacle.position.x - car_x;
        double dy = obstacle.position.y - car_y;
        double distance = std::hypot(dx, dy);
        double angle_to_obstacle = std::atan2(dy, dx);
        double angle_diff = angle_to_obstacle - car_yaw;
        
        angle_diff = std::fmod(angle_diff, 2 * M_PI);
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        else if (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        double abs_angle_diff = std::abs(angle_diff);
        
        if (distance < stop_distance && abs_angle_diff < stop_angle_threshold) {
            current_stop = true;
            break;
        }
        
        if (distance < slow_distance && abs_angle_diff < front_angle_threshold) {
            obstacle_slow = true;
        }
    }

    if (current_stop) {
        obstacle_stop_state_ = true;
        obstacle_stop_start_time_ = now;
    } else if (obstacle_stop_state_) {
        double elapsed = (now - obstacle_stop_start_time_).toSec();
        if (elapsed >= obstacle_stop_duration_) {
            obstacle_stop_state_ = false;
        }
    }

    auto_car_perception::ObstacleState stop_msg;
    stop_msg.state = obstacle_stop_state_;
    obstacle_stop_pub.publish(stop_msg);

    auto_car_perception::ObstacleState slow_msg;
    slow_msg.state = obstacle_slow;
    obstacle_slow_pub.publish(slow_msg);

    ROS_INFO_THROTTLE(1.0, "Obstacle conditions: stop=%s (state), slow=%s (current)",
                     obstacle_stop_state_ ? "true" : "false",
                     obstacle_slow ? "true" : "false");
}

void ObstacleDetectionNode::updateTrackedObstacles(geometry_msgs::PoseArray& obstacles, const ros::Time& stamp) {
    geometry_msgs::PoseArray transformed_obstacles;
    transformed_obstacles.header.stamp = stamp;
    transformed_obstacles.header.frame_id = world_frame;

    for (auto& obstacle : obstacles.poses) {
        geometry_msgs::PointStamped pt_cam, pt_world;
        pt_cam.header.stamp = stamp;
        pt_cam.header.frame_id = camera_frame;
        pt_cam.point.x = obstacle.position.x;
        pt_cam.point.y = obstacle.position.y;
        pt_cam.point.z = obstacle.position.z;

        try {
            tf_buffer.transform(pt_cam, pt_world, world_frame, ros::Duration(0.1));
            geometry_msgs::Pose pose;
            pose.position = pt_world.point;
            pose.orientation.w = 1.0;
            transformed_obstacles.poses.push_back(pose);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }
    }
    
    obstacles = transformed_obstacles;
}

void ObstacleDetectionNode::semanticCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!camera_info_received || !depth_received) {
        ROS_WARN_THROTTLE(2.0, "Waiting for camera info and depth image...");
        return;
    }

    cv::Mat semantic_img;
    try {
        semantic_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error (semantic): %s", e.what());
        return;
    }

    resolution_mismatch = false;
    scale_x = static_cast<double>(depth_image.cols) / semantic_img.cols;
    scale_y = static_cast<double>(depth_image.rows) / semantic_img.rows;

    cv::Mat hsv_img, mask1, mask2, mask;
    cv::cvtColor(semantic_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_img, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, 255, 255), mask1);
    cv::inRange(hsv_img, cv::Scalar(160, s_min, v_min), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    int red_pixels = cv::countNonZero(mask);

    if (red_pixels < 50) {
        geometry_msgs::PoseArray empty_obstacles;
        empty_obstacles.header.stamp = ros::Time::now();
        empty_obstacles.header.frame_id = world_frame;
        obstacles_pub.publish(empty_obstacles);
        
        if (car_pose_received) {
            checkObstacleDistances(empty_obstacles);
        }
        return;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    geometry_msgs::PoseArray obstacles_msg;
    obstacles_msg.header.stamp = msg->header.stamp;
    obstacles_msg.header.frame_id = camera_frame;

    float fx = camera_info.K[0];
    float fy = camera_info.K[4];
    float cx = camera_info.K[2];
    float cy = camera_info.K[5];

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 20) {
            continue;
        }

        cv::Moments mu = cv::moments(c);
        if (mu.m00 < 1e-5) continue;

        double u_sem = mu.m10 / mu.m00;
        double v_sem = mu.m01 / mu.m00;

        double u_depth = u_sem * scale_x;
        double v_depth = v_sem * scale_y;

        int u = static_cast<int>(std::round(u_depth));
        int v = static_cast<int>(std::round(v_depth));
        u = std::max(0, std::min(u, depth_image.cols - 1));
        v = std::max(0, std::min(v, depth_image.rows - 1));

        float z = depth_image.at<float>(v, u);
        if (std::isnan(z) || std::isinf(z) || z < depth_min_valid || z > depth_max_valid) {
            continue;
        }

        float x = (u_depth - cx) * z / fx;
        float y = (v_depth - cy) * z / fy;

        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.w = 1.0;
        obstacles_msg.poses.push_back(pose);
    }

    updateTrackedObstacles(obstacles_msg, msg->header.stamp);

    obstacles_pub.publish(obstacles_msg);
    ROS_INFO_THROTTLE(1.0, "Published %lu obstacles at time %.3f", 
                     obstacles_msg.poses.size(), 
                     obstacles_msg.header.stamp.toSec());
    
    if (car_pose_received) {
        checkObstacleDistances(obstacles_msg);
    } else {
        ROS_WARN_THROTTLE(1.0, "Car pose not available for distance check");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection_node");
    ObstacleDetectionNode node;
    ros::spin();
    return 0;
}
