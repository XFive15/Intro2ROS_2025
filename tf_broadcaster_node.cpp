#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>

class TFBroadcaster
{
public:
    TFBroadcaster()
    {
        // 订阅小车位姿（用于 world → INS）
        pose_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &TFBroadcaster::poseCallback, this);

        // 初始化并发布静态变换：INS → 各相机
        publishStaticTransforms();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    tf2_ros::TransformBroadcaster dynamic_broadcaster_;

    // 发布所有静态变换（只调用一次）
    void publishStaticTransforms()
    {
        std::vector<geometry_msgs::TransformStamped> transforms;

        transforms.push_back(makeStaticTF("INS", "OurCar/Sensors/DepthCamera",     0.82,  0.0, 0.56, -0.5*M_PI,  0.0, -0.5*M_PI));
        transforms.push_back(makeStaticTF("INS", "OurCar/Sensors/RGBCameraLeft",    0.82, -0.2, 0.56, -0.5*M_PI,  0.0, -0.5*M_PI));
        transforms.push_back(makeStaticTF("INS", "OurCar/Sensors/RGBCameraRight",   0.82,  0.2, 0.56, -0.5*M_PI,  0.0, -0.5*M_PI));
        transforms.push_back(makeStaticTF("INS", "OurCar/Sensors/SemanticCamera",   0.82,  0.0, 0.56, -0.5*M_PI,  0.0, -0.5*M_PI));

        static_broadcaster_.sendTransform(transforms);
        ROS_INFO("Static transforms from INS to sensors published.");
    }

    // 构造静态 TF
    geometry_msgs::TransformStamped makeStaticTF(
        const std::string& parent, const std::string& child,
        double x, double y, double z,
        double roll, double pitch, double yaw)
    {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = parent;
        tf_msg.child_frame_id = child;

        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        return tf_msg;
    }

    // 动态变换：world → INS
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;  // 使用消息时间戳，避免 extrapolation
        tf_msg.header.frame_id = "world";
        tf_msg.child_frame_id = "INS";

        tf_msg.transform.translation.x = msg->pose.position.x;
        tf_msg.transform.translation.y = msg->pose.position.y;
        tf_msg.transform.translation.z = msg->pose.position.z;
        tf_msg.transform.rotation = msg->pose.orientation;

        dynamic_broadcaster_.sendTransform(tf_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster_node");
    TFBroadcaster broadcaster;

    ros::spin();  // 仅处理回调即可
    return 0;
}