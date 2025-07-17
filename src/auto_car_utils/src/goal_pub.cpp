#include "goal_pub.h"
#include <cmath>

GoalPublisher::GoalPublisher(const std::vector<geometry_msgs::Point>& goal_points)
    : goal_points_(goal_points)
{
    odom_sub_  = nh_.subscribe("/odom", 10, &GoalPublisher::odomCallback, this);
    goal_pub_  = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    cancel_pub_= nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
    stop_sub_  = nh_.subscribe("/stop", 10, &GoalPublisher::stopCallback, this);
}

void GoalPublisher::odomCallback(const nav_msgs::Odometry::ConstPtr&)
{
    tf::StampedTransform transform;
    try
    {
        listener_.lookupTransform("world", "OurCar/INS", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("TF lookup failed: %s", ex.what());
        return;
    }

    geometry_msgs::Point current_pos;
    current_pos.x = transform.getOrigin().x();
    current_pos.y = transform.getOrigin().y();
    current_pos.z = 0;

    geometry_msgs::Point current_goal = goal_points_[current_goal_index_];

    if (isClose(current_pos, current_goal))
    {
        publishNextGoal();
    }

    if (first_odom_)
    {
        first_odom_ = false;
        publishGoal(current_goal);
    }
}

bool GoalPublisher::isClose(const geometry_msgs::Point& p1,
                            const geometry_msgs::Point& p2) const
{
    return std::hypot(p1.x - p2.x, p1.y - p2.y) <= tolerance_;
}

void GoalPublisher::publishNextGoal()
{
    if (current_goal_index_ < goal_points_.size() - 1)
    {
        ++current_goal_index_;
        publishGoal(goal_points_[current_goal_index_]);
    }
    else
    {
        ROS_INFO("All goals have been reached.");
    }
}

void GoalPublisher::publishGoal(const geometry_msgs::Point& goal_point)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp    = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position   = goal_point;
    pose.pose.orientation.w = 1.0;   // 朝向任意

    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.goal.target_pose = pose;

    goal_pub_.publish(goal_msg);
    ROS_INFO_STREAM("Published goal: (" << goal_point.x << ", "
                                        << goal_point.y << ")");
}

void GoalPublisher::stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data && !stop_)
    {
        stop_ = true;
        cancel_pub_.publish(actionlib_msgs::GoalID());
        ROS_INFO("Stopping the car.");
    }
    else if (!msg->data && stop_)
    {
        stop_ = false;
        publishGoal(goal_points_[current_goal_index_]);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");

    std::vector<geometry_msgs::Point> goals;
    geometry_msgs::Point p;

    p.x = -49; p.y = 46;  goals.push_back(p);
    p.x = 224; p.y = 45;  goals.push_back(p);
    p.x = 228; p.y =  8;  goals.push_back(p);
    p.x = 130; p.y =  3;  goals.push_back(p);
    p.x = 122; p.y = 44;  goals.push_back(p);
    p.x =  51; p.y = 52;  goals.push_back(p);
    p.x =  45; p.y = 10;  goals.push_back(p);
    p.x = -62; p.y =  1;  goals.push_back(p);
    p.x = -61; p.y =-12;  goals.push_back(p);

    GoalPublisher node(goals);
    ros::spin();
    return 0;
}