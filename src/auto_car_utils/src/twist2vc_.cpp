#include "twist2vc.h" 
#include <simulation/VehicleControl.h> 

CmdVelToVehicleControl::CmdVelToVehicleControl(ros::NodeHandle& nh)
    : nh_(nh)
{
    // 订阅安全速度指令
    safe_cmd_vel_sub_ = nh_.subscribe(
        "/safe_cmd_vel", 10, // 修改为安全监控的输出
        &CmdVelToVehicleControl::cmdVelCallback, this);
    
    original_cmd_vel_sub_ = nh_.subscribe(
        "/cmd_vel", 10,
        &CmdVelToVehicleControl::cmdVelCallback, this);

    // 发布车辆控制指令
    vehicle_control_pub_ = nh_.advertise<simulation::VehicleControl>(
        "/car_command", 10);

    ROS_INFO("CmdVelToVehicleControl node initialized.");
}

void CmdVelToVehicleControl::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg)
{
    simulation::VehicleControl control_msg;
    double linear_x = msg->linear.x;

    // 根据速度大小决定控制模式
    if (linear_x > 0.05) {            // 前进
        control_msg.Throttle = linear_x * 0.8;
        control_msg.Brake    = 0.0;
        control_msg.Reserved = 0;
    } else if (std::abs(linear_x) <= 0.05) {  // 停车
        control_msg.Throttle = 0.0;
        control_msg.Brake    = 1.0;
        control_msg.Reserved = 0;
    } else {                         // 倒车
        control_msg.Throttle = linear_x;
        control_msg.Brake    = 0.0;
        control_msg.Reserved = 1;
    }

    // 转向控制
    control_msg.Steering = -msg->angular.z;

    vehicle_control_pub_.publish(control_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_to_vehicle_control");
    ros::NodeHandle nh("~");

    CmdVelToVehicleControl node(nh);

    ros::spin();
    return 0;
}