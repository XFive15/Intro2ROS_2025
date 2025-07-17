#include "safety_monitor.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_monitor");
    ros::NodeHandle nh;
    
    SafetyMonitor monitor(nh);
    
    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
        monitor.update();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}