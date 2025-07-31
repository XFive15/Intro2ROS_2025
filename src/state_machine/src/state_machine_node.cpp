#include "state_machine.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle nh;
    
    SafetyMonitor monitor(nh);
    
    ros::Rate rate(10);  
    while (ros::ok()) {
        monitor.update();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
