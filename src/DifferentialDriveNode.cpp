#include <ros/ros.h>

#include "edubot/DifferentialDrive.h"
#include "ros_wrappers/ROSDifferentialDrive.h"

#define WHEEL_BASE  0.212f // m
#define WHEEL_DIA   0.065f // m

#define WHEEL_MAX_VEL 12.0f // rad/s

DifferentialDrive differential_drive(WHEEL_BASE, WHEEL_DIA / 2, WHEEL_MAX_VEL);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_drive_node");

    ros::NodeHandle node_handle;

    ROSDifferentialDrive ros_differential_drive(differential_drive, node_handle, "");
    
    ros::spin();

    return 0;
}