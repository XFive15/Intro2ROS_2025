#include "twist2vc.h" 
#include <simulation/VehicleControl.h> 

CmdVelToVehicleControl::CmdVelToVehicleControl(ros::NodeHandle& nh)
    : nh_(nh)
{
    // 订阅安全速度指令（用于油门/刹车控制）
    safe_cmd_vel_sub_ = nh_.subscribe(
        "/safe_cmd_vel", 10,
        &CmdVelToVehicleControl::safeCmdVelCallback, this);
    
    // 订阅原始速度指令（用于转向控制）
    original_cmd_vel_sub_ = nh_.subscribe(
        "/cmd_vel", 10,
        &CmdVelToVehicleControl::originalCmdVelCallback, this);

    // 发布车辆控制指令
    vehicle_control_pub_ = nh_.advertise<simulation::VehicleControl>(
        "/car_command", 10);

    // 初始化速度存储
    last_safe_vel_.linear.x = 0.0;
    last_original_vel_.angular.z = 0.0;

    ROS_INFO("CmdVelToVehicleControl node initialized.");
}

void CmdVelToVehicleControl::safeCmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg)
{
    // 存储安全速度指令（用于油门/刹车控制）
    last_safe_vel_ = *msg;
    
    // 更新车辆控制命令
    publishVehicleControl();
}

void CmdVelToVehicleControl::originalCmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg)
{
    // 存储原始速度指令（用于转向控制）
    last_original_vel_.angular.z = msg->angular.z;
    
    // 更新车辆控制命令
    publishVehicleControl();
}

void CmdVelToVehicleControl::publishVehicleControl()
{
    simulation::VehicleControl control_msg;
    double linear_x = last_safe_vel_.linear.x;

    // 根据速度大小决定控制模式
    if (linear_x > 0.05) {            // 前进
        control_msg.Throttle = linear_x * 1.0;
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

    // 转向控制使用原始速度指令的角速度值
    control_msg.Steering = -last_original_vel_.angular.z;

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