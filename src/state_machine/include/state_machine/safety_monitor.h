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
    
    // define state machine funcitons
    void publishZeroVelocity();
    void publishLimitedVelocity();
    void publishNormalVelocity();
    
    // states
    enum class State {
        NORMAL,
        SLOW_DOWN,
        STOP
    };
    
    State current_state_;
    ros::NodeHandle nh_;
    

    bool stop_condition_active_;       // 红灯或障碍物停止
    bool traffic_light_active_;        // 红绿灯存在
    bool obstacle_stop_active_;        // 障碍物停止
    bool obstacle_slow_active_;        // 障碍物减速

    bool prev_stop_condition_;
    bool prev_traffic_light_;
    bool prev_obstacle_stop_;
    bool prev_obstacle_slow_;
    
    
    geometry_msgs::Twist last_cmd_vel_;
    const double SLOW_SPEED = 0.2;     // 减速状态下的最大速度
    
    
    ros::Subscriber stop_sub_;
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber obstacle_stop_sub_;
    ros::Subscriber obstacle_slow_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher safe_cmd_vel_pub_;
};

#endif 