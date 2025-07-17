#ifndef GOAL_PUB_H
#define GOAL_PUB_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <vector>

class GoalPublisher
{
public:
    explicit GoalPublisher(const std::vector<geometry_msgs::Point>& goal_points);

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);
    void publishGoal(const geometry_msgs::Point& goal_point);
    void publishNextGoal();
    bool isClose(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2) const;

    ros::NodeHandle            nh_;
    ros::Subscriber            odom_sub_;
    ros::Publisher             goal_pub_;
    ros::Publisher             cancel_pub_;
    ros::Subscriber            stop_sub_;
    tf::TransformListener      listener_;

    double                     tolerance_{1.0};
    std::vector<geometry_msgs::Point> goal_points_;
    std::size_t                current_goal_index_{0};
    bool                       first_odom_{true};
    bool                       stop_{false};
};

#endif  // GOAL_PUB_H
