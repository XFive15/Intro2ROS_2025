#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class SafetyMonitor
{
public:
    explicit SafetyMonitor(ros::NodeHandle& nh);
    void update(); 
    
private:
    // Callbacks
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);
    void trafficLightCallback(const std_msgs::Bool::ConstPtr& msg);
    void obstacleStopCallback(const std_msgs::Bool::ConstPtr& msg);
    void obstacleSlowCallback(const std_msgs::Bool::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // Velocity publishing functions
    void publishZeroVelocity();
    void publishLimitedVelocity(double max_speed);
    void publishNormalVelocity();
    
    // states
    enum class State {
        NORMAL,
        SLOW_DOWN_TRAFFIC,   // Traffic light slow condition 
        SLOW_DOWN_OBSTACLE,  // Obstacle slow condition 
        STOP
    };
    
    State current_state_;
    ros::NodeHandle nh_;
    
    // Activation flags
    bool stop_condition_active_;       // Red light or emergency stop
    bool traffic_light_active_;        // Traffic light present
    bool obstacle_stop_active_;        // Obstacle requires full stop
    bool obstacle_slow_active_;        // Obstacle requires slow down

    // Previous states for logging
    bool prev_stop_condition_;
    bool prev_traffic_light_;
    bool prev_obstacle_stop_;
    bool prev_obstacle_slow_;
    
    geometry_msgs::Twist last_cmd_vel_;
    
    // ROS communication
    ros::Subscriber stop_sub_;
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber obstacle_stop_sub_;
    ros::Subscriber obstacle_slow_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher safe_cmd_vel_pub_;
    ros::Publisher status_pub_;
};

#endif
