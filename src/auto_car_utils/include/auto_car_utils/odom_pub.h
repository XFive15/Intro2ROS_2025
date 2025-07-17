#ifndef ODOM_PUB_H
#define ODOM_PUB_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TfToOdom
{
public:
    TfToOdom();
    ~TfToOdom();

private:
    void timerCallback(const ros::TimerEvent& event);

    ros::NodeHandle          nh_;
    tf::TransformListener*   listener_{nullptr};
    tf::TransformBroadcaster broadcaster_;
    ros::Publisher           odom_pub_;
    ros::Timer               timer_;
    tf::StampedTransform     transform_;
};

#endif
