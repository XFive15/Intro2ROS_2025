#include "obstacle_detection.h"
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

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
      stop_distance(3),
      slow_distance(40.0),
      front_angle_threshold(20.0 * M_PI / 180.0)
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
    nh_private.param("slow_distance", slow_distance, slow_distance);
    nh_private.param("front_angle_threshold", front_angle_threshold, front_angle_threshold);

    camera_info_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info", 1,
                                   &ObstacleDetectionNode::cameraInfoCallback, this);
    depth_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1,
                             &ObstacleDetectionNode::depthCallback, this);
    semantic_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1,
                                &ObstacleDetectionNode::semanticCallback, this);
    car_pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1,
                                &ObstacleDetectionNode::carPoseCallback, this);

    obstacles_pub = nh.advertise<geometry_msgs::PoseArray>("/obstacles", 10);
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacles_markers", 10);
    obstacle_slow_pub = nh.advertise<std_msgs::Bool>("/obstacle_slow", 10);
    obstacle_stop_pub = nh.advertise<std_msgs::Bool>("/obstacle_stop", 10);

    ROS_INFO("Obstacle Detection Node Initialized");
    ROS_INFO("Using camera frame: %s", camera_frame.c_str());
    ROS_INFO("Using world frame: %s", world_frame.c_str());
    ROS_INFO("Depth valid range: %.1f - %.1f meters", depth_min_valid, depth_max_valid);
    ROS_INFO("Stop distance: %.1f m, Slow distance: %.1f m", stop_distance, slow_distance);
    ROS_INFO("Front angle threshold: %.1f°", front_angle_threshold * 180.0 / M_PI);

    // Wait for TF to be ready
    try {
        tf_buffer.canTransform(world_frame, camera_frame, ros::Time(0), ros::Duration(10.0));
        ROS_INFO("TF from [%s] to [%s] is available", camera_frame.c_str(), world_frame.c_str());
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("TF check failed: %s", ex.what());
    }
}

void ObstacleDetectionNode::carPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_car_pose = *msg;
    car_pose_received = true;
    // ROS_DEBUG("Received car pose: x=%.2f, y=%.2f", 
    //           current_car_pose.pose.position.x, 
    //           current_car_pose.pose.position.y);
}

void ObstacleDetectionNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info = *msg;
    camera_info_received = true;
    // ROS_DEBUG("Received camera info");
}

void ObstacleDetectionNode::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Handle different depth encodings
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image = cv_ptr->image;
        } 
        else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            // Convert from mm to meters
            cv_ptr->image.convertTo(depth_image, CV_32FC1, 0.001);
        }
        else {
            ROS_ERROR("Unsupported depth encoding: %s", msg->encoding.c_str());
            return;
        }

        depth_received = true;

        // Mask out invalid depth values
        cv::Mat mask_invalid = (depth_image > depth_max_valid) | (depth_image < depth_min_valid);
        depth_image.setTo(std::numeric_limits<float>::quiet_NaN(), mask_invalid);

        // Debug info
        double minVal, maxVal;
        cv::minMaxLoc(depth_image, &minVal, &maxVal, nullptr, nullptr);
        // ROS_DEBUG_THROTTLE(1.0, "Depth range: min=%.2fm, max=%.2fm", minVal, maxVal);
        
        int valid_pixels = cv::countNonZero(cv::Mat(depth_image == depth_image));
        size_t total_pixels = depth_image.total();
        // ROS_DEBUG_THROTTLE(1.0, "Valid depth pixels: %d/%zu (%.1f%%)", 
                        //    valid_pixels, total_pixels,
                        //    100.0 * valid_pixels / static_cast<double>(total_pixels));
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error (depth): %s", e.what());
    }
}

void ObstacleDetectionNode::showDebugWindows(const cv::Mat& semantic_img, const cv::Mat& mask) {
    if (!semantic_img.empty()) {
        cv::imshow("Semantic Image", semantic_img);
    }
    
    if (!mask.empty()) {
        cv::Mat mask_bgr;
        cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
        cv::imshow("Red Mask", mask_bgr);
    }
    
    if (!depth_image.empty()) {
        cv::Mat depth_vis;
        double minVal, maxVal;
        cv::minMaxLoc(depth_image, &minVal, &maxVal);
        depth_image.convertTo(depth_vis, CV_8UC1, 255.0/(maxVal-minVal), -minVal*255.0/(maxVal-minVal));
        cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
        cv::imshow("Depth", depth_vis);
    }
    
    cv::waitKey(1);
}

void ObstacleDetectionNode::publishObstacleMarkers(const geometry_msgs::PoseArray& obstacles) {
    visualization_msgs::MarkerArray markers;
    
    // Delete all previous markers
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.header.frame_id = world_frame;
    delete_marker.ns = "obstacles";
    markers.markers.push_back(delete_marker);
    
    // Create sphere markers for each obstacle
    for (size_t i = 0; i < obstacles.poses.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header = obstacles.header;
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = obstacles.poses[i];
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;  // Semi-transparent
        marker.lifetime = ros::Duration(0.5);
        markers.markers.push_back(marker);
    }
    
    markers_pub.publish(markers);
}

void ObstacleDetectionNode::checkObstacleDistances(const geometry_msgs::PoseArray& obstacles) {
    // if (!car_pose_received) {
    //     // ROS_WARN_THROTTLE(1.0, "Car pose not received, cannot check obstacle distances");
    //     return;
    // }

    bool obstacle_stop = false;
    bool obstacle_slow = false;

    // Get car position and orientation
    double car_x = current_car_pose.pose.position.x;
    double car_y = current_car_pose.pose.position.y;
    
    // Extract car yaw from quaternion using tf2::getYaw
    double car_yaw = tf2::getYaw(current_car_pose.pose.orientation);

    for (const auto& obstacle : obstacles.poses) {
        // Calculate obstacle distance from car
        double dx = obstacle.position.x - car_x;
        double dy = obstacle.position.y - car_y;
        double distance = std::hypot(dx, dy);

        // Check stop condition  meters)
        if (distance < stop_distance) {
            obstacle_stop = true;
            ROS_DEBUG("Stop condition: obstacle at %.2fm (threshold: %.2fm)", distance, stop_distance);
        }

        // Check slow condition (within 20 meters and in front of car)
        if (distance < slow_distance) {
            // Calculate obstacle angle relative to car
            double angle_to_obstacle = std::atan2(dy, dx);
            double angle_diff = angle_to_obstacle - car_yaw;
            
            // Normalize angle difference to [-π, π]
            angle_diff = std::fmod(angle_diff, 2 * M_PI);
            if (angle_diff > M_PI) {
                angle_diff -= 2 * M_PI;
            } else if (angle_diff < -M_PI) {
                angle_diff += 2 * M_PI;
            }
            
            double abs_angle_diff = std::abs(angle_diff);
            if (abs_angle_diff < front_angle_threshold) {
                obstacle_slow = true;
                ROS_DEBUG("Slow condition: obstacle at %.2fm, angle diff: %.1f° (threshold: %.1f°)", 
                          distance, abs_angle_diff * 180.0 / M_PI, front_angle_threshold * 180.0 / M_PI);
            }
        }
    }

    // Publish results
    std_msgs::Bool stop_msg;
    stop_msg.data = obstacle_stop;
    obstacle_stop_pub.publish(stop_msg);

    std_msgs::Bool slow_msg;
    slow_msg.data = obstacle_slow;
    obstacle_slow_pub.publish(slow_msg);

    ROS_INFO_THROTTLE(1.0, "Obstacle conditions: stop=%s, slow=%s",
                     obstacle_stop ? "true" : "false",
                     obstacle_slow ? "true" : "false");
}

void ObstacleDetectionNode::semanticCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!camera_info_received || !depth_received) {
        // ROS_WARN_THROTTLE(2.0, "Waiting for camera info and depth image...");
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

    // ROS_DEBUG_THROTTLE(1.0, "Depth: %dx%d, Semantic: %dx%d, Scale: x=%.2f, y=%.2f",
    //                  depth_image.cols, depth_image.rows,
    //                  semantic_img.cols, semantic_img.rows,
    //                  scale_x, scale_y);

    cv::Mat hsv_img, mask1, mask2, mask;
    cv::cvtColor(semantic_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_img, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, 255, 255), mask1);
    cv::inRange(hsv_img, cv::Scalar(160, s_min, v_min), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    showDebugWindows(semantic_img, mask);

    int red_pixels = cv::countNonZero(mask);
    ROS_DEBUG_THROTTLE(1.0, "Red pixels detected: %d", red_pixels);

    if (red_pixels < 50) {
        // ROS_WARN_THROTTLE(1.0, "Not enough red pixels detected: %d", red_pixels);
        // Publish empty obstacles array
        geometry_msgs::PoseArray empty_obstacles;
        empty_obstacles.header.stamp = ros::Time::now();
        empty_obstacles.header.frame_id = world_frame;
        obstacles_pub.publish(empty_obstacles);
        
        // Check distances with empty obstacles
        if (car_pose_received) {
            checkObstacleDistances(empty_obstacles);
        }
        return;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ROS_DEBUG_THROTTLE(1.0, "Found %lu contours", contours.size());

    geometry_msgs::PoseArray obstacles_msg;
    obstacles_msg.header.stamp = ros::Time::now();
    obstacles_msg.header.frame_id = world_frame;

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

        // Clamp coordinates to valid range
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

        geometry_msgs::PointStamped pt_cam;
        pt_cam.header.stamp = msg->header.stamp;
        pt_cam.header.frame_id = camera_frame;
        pt_cam.point.x = x;
        pt_cam.point.y = y;
        pt_cam.point.z = z;

        geometry_msgs::PointStamped pt_world;
        try {
            tf_buffer.transform(pt_cam, pt_world, world_frame, ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            continue;
        }

        geometry_msgs::Pose pose;
        pose.position = pt_world.point;
        pose.orientation.w = 1.0;
        obstacles_msg.poses.push_back(pose);
        
        ROS_DEBUG("Detected obstacle at (%.2f, %.2f, %.2f) in world frame", 
                  pt_world.point.x, pt_world.point.y, pt_world.point.z);
    }

    obstacles_pub.publish(obstacles_msg);
    ROS_INFO_THROTTLE(1.0, "Published %lu obstacles at time %.3f", 
                     obstacles_msg.poses.size(), 
                     obstacles_msg.header.stamp.toSec());
    
    // Publish markers for visualization
    publishObstacleMarkers(obstacles_msg);
    
    // Check obstacle distances relative to car
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