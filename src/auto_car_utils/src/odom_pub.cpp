#include "odom_pub.h"

TfToOdom::TfToOdom()
{
    // 创建 TF 监听器
    listener_ = new tf::TransformListener();

    // 发布 /odom
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

    // 定时器：10 Hz
    timer_ = nh_.createTimer(ros::Duration(0.1),
                             &TfToOdom::timerCallback,
                             this);
}

TfToOdom::~TfToOdom()
{
    delete listener_;
}

void TfToOdom::timerCallback(const ros::TimerEvent&)
{
    // 最多等待 0.5 s 获取 world→OurCar/INS 的 TF
    if (!listener_->waitForTransform("world",
                                     "OurCar/INS",
                                     ros::Time(0),
                                     ros::Duration(0.5)))
    {
        ROS_WARN("Transform not available yet, skipping this cycle.");
        return;
    }

    try
    {
        listener_->lookupTransform("world",
                                   "OurCar/INS",
                                   ros::Time(0),
                                   transform_);
    }
    catch (tf::TransformException& e)
    {
        ROS_WARN("%s", e.what());
        return;
    }

    // 填充 Odometry 消息
    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id  = "OurCar/INS";

    // 位置
    odom.pose.pose.position.x = transform_.getOrigin().x();
    odom.pose.pose.position.y = transform_.getOrigin().y();
    odom.pose.pose.position.z = transform_.getOrigin().z();

    // 姿态（四元数）
    odom.pose.pose.orientation.x = transform_.getRotation().x();
    odom.pose.pose.orientation.y = transform_.getRotation().y();
    odom.pose.pose.orientation.z = transform_.getRotation().z();
    odom.pose.pose.orientation.w = transform_.getRotation().w();

    // 发布 Odometry
    odom_pub_.publish(odom);

    // 再广播一次相同的 TF
    broadcaster_.sendTransform(
        tf::StampedTransform(transform_,
                             ros::Time::now(),
                             "world",
                             "OurCar/INS"));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_pub");

    try
    {
        TfToOdom node;
        ros::spin();
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
    return 0;
}

