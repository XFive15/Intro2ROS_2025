#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <simulation/VehicleControl.h>

class CmdVelToVehicleControl {
public:
    explicit CmdVelToVehicleControl(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber safe_cmd_vel_sub_;
    ros::Subscriber original_cmd_vel_sub_;
    ros::Publisher vehicle_control_pub_;
    
    geometry_msgs::Twist last_safe_vel_;      
    geometry_msgs::Twist last_original_vel_;   
    
    void safeCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void originalCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void publishVehicleControl();
};