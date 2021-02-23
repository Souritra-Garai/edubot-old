#include "ros/ros.h"

#include "edubot/Motor_Controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Motor_Controller_Node");

    ros::NodeHandle node_handle; 

    printf("Motor Control Node initialized\n");

    Motor_Controller motor_L(node_handle, "U1/L");
    Motor_Controller motor_R(node_handle, "U1/R");

    printf("Motor object initialized\n");

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        motor_L.spinOnce();
        motor_R.spinOnce();

        printf("Completed a spin\n");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
