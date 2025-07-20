#include "safety_monitor.h"

SafetyMonitor::SafetyMonitor(ros::NodeHandle& nh) 
    : nh_(nh), 
      current_state_(State::NORMAL),
      stop_condition_active_(false),
      traffic_light_active_(false),
      obstacle_stop_active_(false),
      obstacle_slow_active_(false),
      prev_stop_condition_(false),
      prev_traffic_light_(false),
      prev_obstacle_stop_(false),
      prev_obstacle_slow_(false)
{
    stop_sub_ = nh_.subscribe("/stop", 1, &SafetyMonitor::stopCallback, this);
    traffic_light_sub_ = nh_.subscribe("/traffic_light_presence", 1, &SafetyMonitor::trafficLightCallback, this);
    obstacle_stop_sub_ = nh_.subscribe("/obstacle_stop", 1, &SafetyMonitor::obstacleStopCallback, this);
    obstacle_slow_sub_ = nh_.subscribe("/obstacle_slow", 1, &SafetyMonitor::obstacleSlowCallback, this);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &SafetyMonitor::cmdVelCallback, this);
    
    safe_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/safe_cmd_vel", 1);
    
    ROS_INFO("Safety Monitor initialized with dual slow-down states");
}

void SafetyMonitor::update()
{
    // State transition logic with priority: STOP > TRAFFIC SLOW > OBSTACLE SLOW > NORMAL
    if (stop_condition_active_ || obstacle_stop_active_) {
        current_state_ = State::STOP;
    } else if (traffic_light_active_) {
        current_state_ = State::SLOW_DOWN_TRAFFIC;
    } else if (obstacle_slow_active_) {
        current_state_ = State::SLOW_DOWN_OBSTACLE;
    } else {
        current_state_ = State::NORMAL;
    }

    // State execution logic
    switch (current_state_) {
        case State::NORMAL:
            publishNormalVelocity();
            break;
        case State::SLOW_DOWN_TRAFFIC:
            publishLimitedVelocity(0.18);  // Traffic light slow speed
            break;
        case State::SLOW_DOWN_OBSTACLE:
            publishLimitedVelocity(0.35);  // Obstacle slow speed
            break;
        case State::STOP:
            publishZeroVelocity();
            break;
    }
}

// Callback implementations remain unchanged
void SafetyMonitor::stopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data != prev_stop_condition_) {
        ROS_INFO_COND(msg->data, "Stop condition activated (traffic light)");
        ROS_INFO_COND(!msg->data, "Stop condition deactivated (traffic light)");
        prev_stop_condition_ = msg->data;
    }
    stop_condition_active_ = msg->data;
}

void SafetyMonitor::trafficLightCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data != prev_traffic_light_) {
        ROS_INFO_COND(msg->data, "Traffic light detected");
        ROS_INFO_COND(!msg->data, "Traffic light cleared");
        prev_traffic_light_ = msg->data;
    }
    traffic_light_active_ = msg->data;
}

void SafetyMonitor::obstacleStopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data != prev_obstacle_stop_) {
        ROS_INFO_COND(msg->data, "Obstacle stop condition activated");
        ROS_INFO_COND(!msg->data, "Obstacle stop condition deactivated");
        prev_obstacle_stop_ = msg->data;
    }
    obstacle_stop_active_ = msg->data;
}

void SafetyMonitor::obstacleSlowCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data != prev_obstacle_slow_) {
        ROS_INFO_COND(msg->data, "Obstacle slow condition activated");
        ROS_INFO_COND(!msg->data, "Obstacle slow condition deactivated");
        prev_obstacle_slow_ = msg->data;
    }
    obstacle_slow_active_ = msg->data;
}

void SafetyMonitor::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_vel_ = *msg;
}

// Velocity publishing implementations
void SafetyMonitor::publishZeroVelocity() {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    safe_cmd_vel_pub_.publish(cmd);
}

void SafetyMonitor::publishLimitedVelocity(double max_speed) {
    geometry_msgs::Twist cmd = last_cmd_vel_;
    
    // Limit linear velocity
    if (cmd.linear.x > max_speed) {
        cmd.linear.x = max_speed;
    }
    
    // Limit angular velocity (unchanged from original)
    const double MAX_ANGULAR = 0.5;
    if (std::abs(cmd.angular.z) > MAX_ANGULAR) {
        cmd.angular.z = (cmd.angular.z > 0) ? MAX_ANGULAR : -MAX_ANGULAR;
    }
    
    safe_cmd_vel_pub_.publish(cmd);
}

void SafetyMonitor::publishNormalVelocity() {
    safe_cmd_vel_pub_.publish(last_cmd_vel_);
}